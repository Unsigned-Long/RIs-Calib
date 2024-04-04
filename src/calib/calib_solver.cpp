// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "calib/calib_solver.h"
#include "calib/spline_estimator.h"
#include "tiny-viewer/core/multi_viewer.h"
#include "pangolin/display/display.h"

namespace ns_ris {

    // -----------
    // CalibSolver
    // -----------

    const std::string CalibSolver::VIEW_SENSORS = "VIEW_SENSORS";
    const std::string CalibSolver::VIEW_SPLINE = "VIEW_SPLINE";

    CalibSolver::CalibSolver(CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager)
            : _dataManager(std::move(calibDataManager)), _paramManager(std::move(calibParamManager)),
              _ceresOption(SplineEstimator::DefaultSolverOptions(Configor::Preference::ThreadsToUse)),
              _solveFinished(false) {
        _trajectory = Trajectory::Create(
                Configor::Prior::SplineKnotTimeDistance,
                _dataManager->GetAlignedStartTimestamp(), _dataManager->GetAlignedEndTimestamp()
        );

        // view create
        ns_viewer::MultiViewerConfigor viewConfig({VIEW_SENSORS, VIEW_SPLINE}, "RIs-Calib");
        viewConfig.grid.at(VIEW_SENSORS).showGrid = false;
        viewConfig.grid.at(VIEW_SENSORS).showIdentityCoord = false;
        viewConfig.WithScreenShotSaveDir(Configor::DataStream::OutputPath);
        for (const auto &name: viewConfig.subWinNames) {
            viewConfig.camera.at(name).initPos = {2.0f, 2.0f, 2.0f};
            viewConfig.grid.at(name).cellSize = 0.1;
        }
        _viewer = ns_viewer::MultiViewer::Create(viewConfig);
        _viewer->RunInMultiThread();

        _ceresOption.callbacks.push_back(new RIsCeresViewerCallBack(_paramManager, _viewer, _trajectory));
        _ceresOption.update_state_every_iteration = true;
        if (Configor::Preference::OutputParamInEachIter) {
            _ceresOption.callbacks.push_back(new RIsCeresDebugCallBack(_paramManager));
        }
    }

    CalibSolver::Ptr CalibSolver::Create(const CalibDataManager::Ptr &calibDataManager,
                                         const CalibParamManager::Ptr &calibParamManager) {
        return std::make_shared<CalibSolver>(calibDataManager, calibParamManager);
    }

    void CalibSolver::Process() {
        spdlog::info("initialization...");
        Initialization();
        _paramManager->ShowParamStatus();

        int radarOptOption = OptOption::NONE, acceOptOption = OptOption::NONE, gyroOptOption = OptOption::NONE;
        for (int i = 0; i < 3; ++i) {
            spdlog::info("batch optimization ({})...", i);
            switch (i) {
                case 0:
                    radarOptOption = OptOption::OPT_SO3 | OptOption::OPT_VEL |
                                     OptOption::OPT_SO3_RjToBc | OptOption::OPT_POS_RjInBc;

                    acceOptOption = OptOption::OPT_SO3 | OptOption::OPT_VEL | OptOption::OPT_GRAVITY |
                                    OptOption::OPT_SO3_BiToBc | OptOption::OPT_POS_BiInBc;

                    gyroOptOption = OptOption::OPT_SO3 | OptOption::OPT_SO3_BiToBc;
                    break;
                case 1:
                    if (!Configor::Preference::OptTemporalParams) {
                        break;
                    }
                    // In this optimization, we add temporal parameters to estimator
                    radarOptOption |= OptOption::OPT_TIME_OFFSET_RjToBc;

                    acceOptOption |= OptOption::OPT_TIME_OFFSET_BiToBc;

                    gyroOptOption |= OptOption::OPT_TIME_OFFSET_BiToBc;
                    break;
                case 2:
                    if (!Configor::Preference::OptIntrinsicParams) {
                        break;
                    }
                    // In this optimization, we add IMU intrinsic parameters to estimator
                    acceOptOption |= OptOption::OPT_ACCE_BIAS;

                    gyroOptOption |= OptOption::OPT_GYRO_BIAS;
                    // gyroOptOption |= OptOption::OPT_GYRO_BIAS | OptOption::OPT_SO3_AtoG;
                    break;
                default:
                    break;
            }
            BatchOptimization(radarOptOption, acceOptOption, gyroOptOption);
            _paramManager->ShowParamStatus();
        }
        _solveFinished = true;
    }

    void CalibSolver::Initialization() {
        // ------------------------------
        // step1: fit rotational b-spline
        // ------------------------------
        spdlog::info("fit rotational b-spline...");

        auto estimator = SplineEstimator::Create(_trajectory, _paramManager);
        for (const auto &[imuTopic, mes]: _dataManager->GetIMUMeasurements()) {
            for (const auto &item: mes) {
                estimator->AddIMUGyroMeasurement(
                        item, imuTopic, OptOption::OPT_SO3 | OptOption::OPT_SO3_BiToBc,
                        Configor::Prior::Weight::GyroWeight
                );
            }
        }
        estimator->FixFirSO3ControlPoint();
        estimator->AddSO3Centralization(_paramManager->EXTRI.SO3_BiToBc_AddressVec(), 1E4, OptOption::OPT_SO3_BiToBc);
        auto sum1 = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum1.BriefReport());

        // --------------------------------
        // step2: extrinsics initialization
        // --------------------------------
        spdlog::info("extrinsics initialization...");
        estimator = SplineEstimator::Create(_trajectory, _paramManager);
        for (const auto &[radarTopic, radarMes]: _dataManager->GetRadarMeasurements()) {
            for (int i = 0; i < static_cast<int>(radarMes.size()) - 1; ++i) {
                int j = i + 1;
                const auto &arrayI = radarMes.at(i);
                const auto &arrayJ = radarMes.at(j);
                // to estimate the radar velocity by linear least-squares solver
                // the minim targets number is 3
                if (arrayI->GetTargets().size() < 3 || arrayJ->GetTargets().size() < 3) { continue; }

                double ti = arrayI->GetTimestamp(), tj = arrayJ->GetTimestamp();

                Eigen::Vector3d vI = arrayI->RadarVelocityFromStaticTargetArray();
                Eigen::Vector3d vJ = arrayJ->RadarVelocityFromStaticTargetArray();

                for (const auto &[imuTopic, imuMes]: _dataManager->GetIMUMeasurements()) {
                    // this factor is fine!!!
                    estimator->AddDiscreteVelPreIntegration(
                            imuMes, imuTopic, radarTopic, ti, tj, vI, vJ,
                            OptOption::OPT_GRAVITY | OptOption::OPT_POS_BiInBc |
                            OptOption::OPT_SO3_RjToBc | OptOption::OPT_POS_RjInBc,
                            Configor::Prior::Weight::VelPIMWeight
                    );
                }
            }
        }
        estimator->AddPOSCentralization(_paramManager->EXTRI.POS_BiInBc_AddressVec(), 1E4, OptOption::OPT_POS_BiInBc);
        auto sum2 = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum2.BriefReport());

        // ----------------------------------------------------
        // step3: extrinsics refinement & velocity b-spline
        // ----------------------------------------------------
        // if do not initialize the velocity bspline in initialization procedure, jump this step
        if (!Configor::Preference::InitVelocityBspline) {
            return;
        }

        spdlog::info("extrinsics refinement & velocity b-spline recovery...");

        estimator = SplineEstimator::Create(_trajectory, _paramManager);
        // ----------------------------------------------------------------------------------
        // adding radar factors in initialization is probably not a good choice,  since the
        // time offset has not been initialized, and using two asynchronous data (i.e., radar
        // data and imu data) in an estimator would lead to bad convergence
        // ----------------------------------------------------------------------------------
        for (const auto &[radarTopic, mes]: _dataManager->GetRadarMeasurements()) {
            for (const auto &targetAry: mes) {
                for (const auto &tar: targetAry->GetTargets()) {
                    estimator->AddRadarMeasurement(
                            tar, radarTopic,
                            OptOption::OPT_VEL | OptOption::OPT_SO3_RjToBc | OptOption::OPT_POS_RjInBc,
                            Configor::Prior::Weight::RadarWeight
                    );
                }
            }
        }
        for (double t = _dataManager->GetAlignedStartTimestamp(); t < _dataManager->GetAlignedEndTimestamp();) {
            double ti = t, tj = t + Configor::Prior::SplineKnotTimeDistance;
            for (const auto &[imuTopic, imuMes]: _dataManager->GetIMUMeasurements()) {
                estimator->AddVelPreIntegration(
                        imuMes, imuTopic, ti, tj,
                        OptOption::OPT_GRAVITY | OptOption::OPT_VEL | OptOption::OPT_POS_BiInBc,
                        Configor::Prior::Weight::VelPIMWeight
                );
            }
            t += Configor::Prior::SplineKnotTimeDistance;
        }
        estimator->AddPOSCentralization(_paramManager->EXTRI.POS_BiInBc_AddressVec(), 1E4, OptOption::OPT_POS_BiInBc);
        auto sum3 = estimator->Solve(_ceresOption);

        spdlog::info("here is the summary:\n{}\n", sum3.BriefReport());
    }

    void CalibSolver::BatchOptimization(int radarOptOption, int acceOptOption, int gyroOptOption) {
        auto GetOptString = [](int opt) -> std::string {
            std::stringstream stringStream;
            stringStream << OptOption::Option(opt);
            return stringStream.str();
        };
        spdlog::info("      radar opt option: {}", GetOptString(radarOptOption));
        spdlog::info("accelerator opt option: {}", GetOptString(acceOptOption));
        spdlog::info("  gyroscope opt option: {}", GetOptString(gyroOptOption));

        int IMUFactorCount = 0, radarFactorCount = 0;
        auto estimator = SplineEstimator::Create(_trajectory, _paramManager);
        for (const auto &[topic, mes]: _dataManager->GetIMUMeasurements()) {
            for (const auto &item: mes) {
                // gyro factors
                estimator->AddIMUGyroMeasurement(item, topic, gyroOptOption, Configor::Prior::Weight::GyroWeight);
                // acce factors
                estimator->AddIMUAcceMeasurement(item, topic, acceOptOption, Configor::Prior::Weight::AcceWeight);
                ++IMUFactorCount;
            }
        }
        for (const auto &[topic, mes]: _dataManager->GetRadarMeasurements()) {
            for (const auto &targetAry: mes) {
                for (const auto &tar: targetAry->GetTargets()) {
                    // radar factors
                    estimator->AddRadarMeasurement(tar, topic, radarOptOption, Configor::Prior::Weight::RadarWeight);
                    ++radarFactorCount;
                }
            }
        }
        spdlog::info("IMU factor count: {}, radar factor count: {}", IMUFactorCount, radarFactorCount);
        estimator->FixFirSO3ControlPoint();
        // Central factors
        estimator->AddSO3Centralization(
                _paramManager->EXTRI.SO3_BiToBc_AddressVec(), 1E4, OptOption::OPT_SO3_BiToBc
        );
        estimator->AddPOSCentralization(
                _paramManager->EXTRI.POS_BiInBc_AddressVec(), 1E4, OptOption::OPT_POS_BiInBc
        );
        estimator->AddTimeOffsetCentralization(
                _paramManager->TEMPORAL.TIME_OFFSET_BiToBc_AddressVec(), 1E4, OptOption::OPT_TIME_OFFSET_BiToBc
        );

        if (Configor::Preference::OutputLMEquationGraph) {
            // save the lm equation
            SaveEquationGraph(estimator);
        }

        auto sum = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }

    void CalibSolver::VisualizationSensorSuite() const {
        static int id = 0;
        auto viewerConfig = ns_viewer::ViewerConfigor("sensor suite " + std::to_string(id++));
        viewerConfig.WithScreenShotSaveDir(Configor::DataStream::OutputPath);
        viewerConfig.camera.initPos = {1.0f, 1.0f, 1.0f};

        ns_viewer::Viewer viewer(viewerConfig);
        viewer.RemoveEntity();
        _paramManager->VisualizationSensors(viewer);
        viewer.RunInSingleThread();
    }

    void CalibSolver::VisualizationBSplines() const {
        static int id = 0;
        auto viewerConfig = ns_viewer::ViewerConfigor("rotation & velocity b-splines " + std::to_string(id++));
        viewerConfig.WithScreenShotSaveDir(Configor::DataStream::OutputPath);
        viewerConfig.camera.initPos = {2.0f, 2.0f, 2.0f};
        viewerConfig.grid.cellSize = 0.1;

        ns_ctraj::Viewer viewer(viewerConfig);
        _trajectory->Visualization(viewer, 0.01, 0.03);
        viewer.RunInSingleThread();
    }

    void CalibSolver::SaveEquationGraph(const SplineEstimator::Ptr &estimator) {
        const static std::string dir = Configor::DataStream::OutputPath + "/lm_equ_graph";
        static int count = 0;
        if (count == 0) {
            std::filesystem::remove_all(dir);
            std::filesystem::create_directory(dir);
        }
        if (!std::filesystem::exists(dir)) {
            spdlog::warn("the directory to save lm equation image (i.e., {}) is invalid!", dir);
        } else {
            // save figure
            // auto lmEquation = estimator->Evaluate(
            //         {_paramManager->EXTRI.GetParamAddressWithDesc(),
            //          _paramManager->INTRI.GetParamAddressWithDesc(),
            //          _paramManager->TEMPORAL.GetParamAddressWithDesc(),
            //          {{_paramManager->GRAVITY.data(), "GRAVITY"}}}
            // );
            // auto lmEquation = estimator->Evaluate(
            //         {GetCTrajSO3KnotAddressWithDesc(_trajectory),
            //          GetCTrajVelKnotAddressWithDesc(_trajectory)}
            // );
            auto lmEquation = estimator->Evaluate(
                    {_paramManager->EXTRI.GetParamAddressWithDesc(),
                     _paramManager->INTRI.GetParamAddressWithDesc(),
                     _paramManager->TEMPORAL.GetParamAddressWithDesc(),
                     {{_paramManager->GRAVITY.data(), "GRAVITY"}},
                     GetCTrajSO3KnotAddressWithDesc(_trajectory),
                     GetCTrajVelKnotAddressWithDesc(_trajectory)}
            );

            {
                // equation
                std::string namePrefix = dir + "/batch_opt_" + std::to_string(count);
                cv::imwrite(namePrefix + ".png",
                            lmEquation.SaveParamBlockOrderListToDisk(
                                    namePrefix + ns_ris::Configor::GetFormatExtension(),
                                    Configor::Preference::OutputDataFormat
                            ).EquationGraph());
            }

            {
                // residuals
                std::ofstream file(
                        dir + "/residuals_" + std::to_string(count) + ns_ris::Configor::GetFormatExtension(),
                        std::ios::out
                );
                std::map<std::size_t, std::string> idToName;
                idToName.insert({IMUAcceFactor<Configor::Prior::SplineOrder>::TypeHashCode(), "IMUAcceFactor"});
                idToName.insert({IMUGyroFactor<Configor::Prior::SplineOrder>::TypeHashCode(), "IMUGyroFactor"});
                idToName.insert({RadarFactor<Configor::Prior::SplineOrder>::TypeHashCode(), "RadarFactor"});

                std::map<std::string, std::vector<Eigen::VectorXd>> data;

                for (const auto &[typeId, residuals]: lmEquation.GetResidualsMap()) {
                    if (idToName.find(typeId) == idToName.cend()) { continue; }
                    std::vector<Eigen::VectorXd> valVec(residuals.size());
                    for (int i = 0; i < static_cast<int>(residuals.size()); ++i) {
                        valVec.at(i) = residuals.at(i);
                    }
                    data.insert({idToName.at(typeId), valVec});
                }
                auto ar = GetOutputArchiveVariant(file, Configor::Preference::OutputDataFormat);
                SerializeByOutputArchiveVariant(
                        ar, Configor::Preference::OutputDataFormat, cereal::make_nvp("residuals", data)
                );
            }

            ++count;
        }
    }

    std::map<const double *, std::string>
    CalibSolver::GetCTrajVelKnotAddressWithDesc(const Trajectory::Ptr &traj) {
        std::map<const double *, std::string> infoMap;
        for (int i = 0; i < std::min(16, int(traj->NumKnots())); ++i) {
            std::string identify = fmt::format("P{:0" + std::to_string((int) log10(traj->NumKnots()) + 1) + "}", i);
            infoMap.insert({traj->GetKnotPos(i).data(), "VEL[" + identify + "]"});
        }
        return infoMap;
    }

    std::map<const double *, std::string>
    CalibSolver::GetCTrajSO3KnotAddressWithDesc(const Trajectory::Ptr &traj) {
        std::map<const double *, std::string> infoMap;
        for (int i = 0; i < std::min(16, int(traj->NumKnots())); ++i) {
            std::string identify = fmt::format("P{:0" + std::to_string((int) log10(traj->NumKnots()) + 1) + "}", i);
            infoMap.insert({traj->GetKnotSO3(i).data(), "SO3[" + identify + "]"});
        }
        return infoMap;
    }

    void CalibSolver::SaveBSplines(int hz) const {
        std::string saveDir = Configor::DataStream::OutputPath + "/splines";
        std::filesystem::remove_all(saveDir);
        if (!std::filesystem::create_directories(saveDir)) {
            throw ns_ris::Status(
                    ns_ris::Status::Flag::ERROR,
                    fmt::format("create directory to save trajectories failed: '{}'", saveDir)
            );
        }
        {
            // sampled points
            auto seq = this->_trajectory->Sampling(1.0 / hz);
            if (!SavePoseSequence(seq, saveDir + "/b-splines-sample" + ns_ris::Configor::GetFormatExtension(),
                                  ns_ris::Configor::Preference::OutputDataFormat)) {
                throw ns_ris::Status(
                        ns_ris::Status::Flag::ERROR,
                        fmt::format("error occurs when saving b-splines to '{}'", saveDir)
                );
            }
        }
        {
            // control points
            std::ofstream file(saveDir + "/b-splines-cp" + ns_ris::Configor::GetFormatExtension());
            auto ar = GetOutputArchiveVariant(file, ns_ris::Configor::Preference::OutputDataFormat);
            SerializeByOutputArchiveVariant(
                    ar, ns_ris::Configor::Preference::OutputDataFormat,
                    cereal::make_nvp("trajectory", *_trajectory)
            );
        }
    }

    void CalibSolver::Visualization() const {
        static int id = 0;
        const std::string win1 = "sensor-suite", win2 = "b-splines";
        auto config = ns_viewer::MultiViewerConfigor(
                {win1, win2}, "sensor-suite & b-splines " + std::to_string(id++)
        );
        config.WithScreenShotSaveDir(Configor::DataStream::OutputPath);
        for (const auto &name: config.subWinNames) {
            config.camera.at(name).initPos = {2.0f, 2.0f, 2.0f};
            config.grid.at(name).cellSize = 0.1;
        }
        config.grid.at(win1).showIdentityCoord = false;
        config.grid.at(win1).showGrid = false;

        ns_viewer::MultiViewer viewer(config);
        {
            auto SE3_BcToBc = Sophus::SE3f();
            auto centerIMU = ns_viewer::IMU::Create(
                    ns_viewer::Posef(SE3_BcToBc.so3().matrix(), SE3_BcToBc.translation()), 0.1,
                    ns_viewer::Colour(0.3f, 0.3f, 0.3f, 1.0f)
            );
            viewer.AddEntity(centerIMU, win1);

            for (const auto &[imuTopic, _]: _paramManager->EXTRI.SO3_BiToBc) {
                auto BiToBc = _paramManager->EXTRI.SE3_BiToBc(imuTopic).cast<float>();
                auto imu = ns_viewer::IMU::Create(
                        ns_viewer::Posef(BiToBc.so3().matrix(), BiToBc.translation()), 0.1
                );
                auto line = ns_viewer::Line::Create(
                        Eigen::Vector3f::Zero(), BiToBc.translation().cast<float>(), ns_viewer::Colour::Black()
                );
                viewer.AddEntity({imu, line}, win1);
            }

            for (const auto &[radarTopic, _]: _paramManager->EXTRI.SO3_RjToBc) {
                auto RjToBc = _paramManager->EXTRI.SE3_RjToBc(radarTopic).cast<float>();
                auto radar = ns_viewer::Radar::Create(
                        ns_viewer::Posef(RjToBc.so3().matrix(), RjToBc.translation()), 0.1, ns_viewer::Colour::Blue()
                );
                auto line = ns_viewer::Line::Create(
                        Eigen::Vector3f::Zero(), RjToBc.translation().cast<float>(), ns_viewer::Colour::Black()
                );
                viewer.AddEntity({radar, line}, win1);
            }
        }
        {
            for (const auto &item: _trajectory->Sampling(0.01)) {
                viewer.AddEntity(ns_viewer::Coordinate::Create(
                        ns_viewer::Posed(item.so3.matrix(), item.t).cast<float>(), 0.03f), win2);
            }
        }
        viewer.RunInSingleThread();
    }

    CalibSolver::~CalibSolver() {
        // solving is not performed or not finished as an exception is thrown
        if (!_solveFinished) { pangolin::QuitAll(); }
            // solving is finished
        else while (_viewer->IsActive()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    }

    // ---------------------
    // RIsCeresDebugCallBack
    // ---------------------

    RIsCeresDebugCallBack::RIsCeresDebugCallBack(CalibParamManager::Ptr calibParamManager)
            : _calibParamManager(std::move(calibParamManager)) {}

    auto RIsCeresDebugCallBack::Create(const CalibParamManager::Ptr &calibParamManager) {
        return new RIsCeresDebugCallBack(calibParamManager);
    }

    ceres::CallbackReturnType RIsCeresDebugCallBack::operator()(const ceres::IterationSummary &summary) {
        // for drawing
        const static std::string paramDir = Configor::DataStream::OutputPath + "/params_iter";
        const std::string iterInfoFilename = paramDir + "/iter_info.csv";

        static int count = 0;
        if (count == 0) {
            std::filesystem::remove_all(paramDir);
            std::filesystem::create_directory(paramDir);

            std::ofstream file(iterInfoFilename, std::ios::out);
            file << "cost,gradient,tr_radius(1/lambda)" << std::endl;
            file.close();
        }
        if (!std::filesystem::exists(paramDir)) {
            spdlog::warn("the directory to save param files (i.e., {}) is invalid!", paramDir);
        } else {
            // save param
            const std::string paramFilename =
                    paramDir + "/params_" + std::to_string(count) + ns_ris::Configor::GetFormatExtension();
            _calibParamManager->Save(paramFilename, ns_ris::Configor::Preference::OutputDataFormat);

            // save iter info
            std::ofstream file(iterInfoFilename, std::ios::app);
            file << count << ',' << summary.cost << ','
                 << summary.gradient_norm << ',' << summary.trust_region_radius << std::endl;
            file.close();

            ++count;
        }
        return ceres::SOLVER_CONTINUE;
    }

    // ----------------------
    // RIsCeresViewerCallBack
    // ----------------------
    RIsCeresViewerCallBack::RIsCeresViewerCallBack(CalibParamManager::Ptr calibParamManager,
                                                   ns_viewer::MultiViewer::Ptr viewer,
                                                   RIsCeresViewerCallBack::Trajectory::Ptr traj)
            : _parMagr(std::move(calibParamManager)), _viewer(std::move(viewer)), _traj(std::move(traj)) {}

    ceres::CallbackReturnType RIsCeresViewerCallBack::operator()(const ceres::IterationSummary &summary) {
        _viewer->RemoveEntity(_idVec, CalibSolver::VIEW_SENSORS);
        _viewer->RemoveEntity(_idVec, CalibSolver::VIEW_SPLINE);

        _idVec = AddEntitiesToSensorViewer();
        auto ids = AddEntitiesToSplineViewer();
        _idVec.insert(_idVec.cbegin(), ids.cbegin(), ids.cend());

        return ceres::CallbackReturnType::SOLVER_CONTINUE;
    }

    auto RIsCeresViewerCallBack::Create(const CalibParamManager::Ptr &calibParamManager,
                                        const ns_viewer::MultiViewer::Ptr &viewer,
                                        const CalibSolver::Trajectory::Ptr &traj) {
        return std::make_shared<RIsCeresViewerCallBack>(calibParamManager, viewer, traj);
    }

    std::vector<std::size_t> RIsCeresViewerCallBack::AddEntitiesToSplineViewer() {
        // spline  viewer
        std::vector<ns_viewer::Entity::Ptr> entities;
        const double st = _traj->MinTime() + Configor::Prior::TimeOffsetPadding;
        const double et = _traj->MaxTime() - Configor::Prior::TimeOffsetPadding;
        for (const auto &item: _traj->Sampling(0.001, st, et)) {
            entities.push_back(
                    ns_viewer::Coordinate::Create(ns_viewer::Posed(item.so3.matrix(), item.t).cast<float>(), 0.04f)
            );
        }
        // const auto &knots = _traj->GetPosSpline().GetKnots();
        // auto knotSize = knots.size();
        // for (std::size_t i = 0; i < knotSize - 1; ++i) {
        //     const auto &ki = knots.at(i);
        //     const auto &kj = knots.at(i + 1);
        //     // lines between knots
        //     auto line = ns_viewer::Line::Create(ki.cast<float>(), kj.cast<float>(), ns_viewer::Colour::Black());
        //     // knots
        //     auto ks = ns_viewer::Landmark::Create(ki.cast<float>(), 0.01f, ns_viewer::Colour::Black());
        //     entities.insert(entities.cend(), {ks, line});
        // }
        // the last knot
        // auto k = ns_viewer::Landmark::Create(knots.back().cast<float>(), 0.01f, ns_viewer::Colour::Black());
        // entities.push_back(k);
        return _viewer->AddEntity(entities, CalibSolver::VIEW_SPLINE);
    }

    std::vector<std::size_t> RIsCeresViewerCallBack::AddEntitiesToSensorViewer() {
        // sensor suites viewer
        std::vector<ns_viewer::Entity::Ptr> entities;
        auto SE3_BcToBc = Sophus::SE3f();
        auto centerIMU = ns_viewer::IMU::Create(
                ns_viewer::Posef(SE3_BcToBc.so3().matrix(), SE3_BcToBc.translation()), 0.1,
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 1.0f)
        );
        entities.push_back(centerIMU);

        for (const auto &[imuTopic, _]: _parMagr->EXTRI.SO3_BiToBc) {
            auto BiToBc = _parMagr->EXTRI.SE3_BiToBc(imuTopic).cast<float>();
            auto imu = ns_viewer::IMU::Create(
                    ns_viewer::Posef(BiToBc.so3().matrix(), BiToBc.translation()), 0.1
            );
            auto line = ns_viewer::Line::Create(
                    Eigen::Vector3f::Zero(), BiToBc.translation().cast<float>(), ns_viewer::Colour::Black()
            );
            entities.insert(entities.cend(), {imu, line});
        }

        for (const auto &[radarTopic, _]: _parMagr->EXTRI.SO3_RjToBc) {
            auto RjToBc = _parMagr->EXTRI.SE3_RjToBc(radarTopic).cast<float>();
            auto radar = ns_viewer::Radar::Create(
                    ns_viewer::Posef(RjToBc.so3().matrix(), RjToBc.translation()), 0.1,
                    ns_viewer::Colour::Blue()
            );
            auto line = ns_viewer::Line::Create(
                    Eigen::Vector3f::Zero(), RjToBc.translation().cast<float>(), ns_viewer::Colour::Black()
            );
            entities.insert(entities.cend(), {radar, line});
        }
        return _viewer->AddEntity(entities, CalibSolver::VIEW_SENSORS);
    }
}