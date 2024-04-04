// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_RADAR_MODEL_HPP
#define RIS_CALIB_RADAR_MODEL_HPP

#include "ctraj/core/simu_trajectory.h"
#include "calib/spline_estimator.h"

namespace ns_ris {
    struct RadarModelLearner {
    public:
        static Sophus::SE3d GeneratePoseBias() {
            constexpr double DEG_TO_RAD = M_PI / 180.0;
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> aRand(-180, 180), pRand(-0.5, 0.5);
            auto a1 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
            auto a2 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
            auto a3 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
            return {(a1 * a2 * a3).toRotationMatrix(), Eigen::Vector3d(pRand(engine), pRand(engine), pRand(engine))};
        }

        static auto ExtractRange(const std::vector<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        static void Learn() {
            auto trajBtoB0 = ns_ctraj::SimuWaveMotion2<4>(2.0, 0.5, 0.0, 2 * M_PI, 1000.0);
            auto BtoB0 = *trajBtoB0.GetTrajectory();
            auto SE3_RtoB = GeneratePoseBias();
            Eigen::Vector3d TinB0(1, 3, 5);
            auto TinR = SE3_RtoB.inverse() * (!BtoB0) * Sophus::SE3d(Sophus::SO3d(), TinB0);

            for (const auto &item: BtoB0.Sampling(0.1)) {
                double t = item.timeStamp;
                // -----------
                // radar model
                // -----------
                Eigen::Vector3d v1 = -Sophus::SO3d::hat(BtoB0.Pose(t).so3() * SE3_RtoB.translation()) *
                                     BtoB0.AngularVeloInRef(t);
                Eigen::Vector3d v2 = BtoB0.LinearVeloInRef(t);
                Sophus::SO3d SO3_B0ToB = SE3_RtoB.so3().inverse() * BtoB0.Pose(t).so3().inverse();
                Eigen::Vector3d v3 = SO3_B0ToB * (v1 + v2);

                Eigen::Vector3d v4 = Sophus::SO3d::hat(TinR.Pose(t).translation()) * (
                        SE3_RtoB.so3().inverse() * BtoB0.Pose(t).so3().inverse() *
                        BtoB0.AngularVeloInRef(t));

                Eigen::Vector3d v5 = SE3_RtoB.so3().inverse()
                                     * (Sophus::SO3d::hat(SE3_RtoB.translation()) *
                                        (BtoB0.Pose(t).so3().inverse() * BtoB0.AngularVeloInRef(t)));

                Eigen::Vector3d v6 = SE3_RtoB.so3().inverse() * BtoB0.Pose(t).so3().inverse() *
                                     BtoB0.LinearVeloInRef(t);
                Eigen::Vector3d v7 = v4 + v5 - v6;

                // r * v_r
                auto mes = BtoB0.RadarStaticMeasurement(t, TinB0, SE3_RtoB);
                std::cout << -TinR.Pose(t).translation().transpose() * v3 << std::endl;
                std::cout << TinR.Pose(t).translation().transpose() * v7 << std::endl;
                std::cout << mes(0) * mes(3) << std::endl << std::endl;
            }
            auto calibParam = CalibParamManager::Create();
            auto imuData = BtoB0.ComputeIMUMeasurement(calibParam->GRAVITY, 1 / 10000.0);
            auto poseVec = BtoB0.Sampling(0.1);
            for (int i = 0; i < static_cast<int>(poseVec.size()) - 1; ++i) {
                int j = i + 1;
                double ti = poseVec.at(i).timeStamp;
                double tj = poseVec.at(j).timeStamp;
                auto [sIter, eIter] = ExtractRange(imuData, ti, tj);
                std::vector<std::pair<double, Eigen::Vector3d>> reorganizedSubData;
                for (auto iter = sIter; iter != eIter; ++iter) {
                    const auto &frame = *iter;
                    double t = frame->GetTimestamp();
                    reorganizedSubData.emplace_back(t, BtoB0.Pose(t).so3() * frame->GetAcce());
                }
                Eigen::Vector3d v1 = TrapIntegrationOnce(reorganizedSubData);
                Eigen::Vector3d v2 = BtoB0.LinearVeloInRef(tj);
                Eigen::Vector3d v3 = BtoB0.LinearVeloInRef(ti);
                Eigen::Vector3d v4 = calibParam->GRAVITY * (tj - ti);
                std::cout << v1.transpose() << std::endl;
                std::cout << (v2 - v3 - v4).transpose() << std::endl << std::endl;
            }
        }
    };

    struct RIsInitLearner {
    public:

        static std::pair<std::vector<RadarTargetArray::Ptr>, std::vector<Eigen::Vector3d>>
        SimuRadarData(ns_ctraj::Trajectory<4> &traj, const Sophus::SE3d &bias) {
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> u(-3, 3);
            std::vector<RadarTargetArray::Ptr> data;
            std::vector<Eigen::Vector3d> targets;
            for (int i = 0; i < 100; ++i) {
                Eigen::Vector3d tar(u(engine), u(engine), u(engine));
                targets.push_back(tar);
            }
            for (const auto &item: traj.Sampling(0.1)) {
                double t = item.timeStamp;
                std::vector<RadarTarget::Ptr> obv;
                for (const auto &tar: targets) {
                    obv.push_back(RadarTarget::Create(t, traj.RadarStaticMeasurement(t, tar, bias)));
                }
                data.push_back(RadarTargetArray::Create(t, obv));
            }

            return {data, targets};
        }

        static Eigen::Vector3d RadarVelocityFromStaticTargetArray(const RadarTargetArray::Ptr &array) {
            Eigen::VectorXd lVec(array->GetTargets().size());
            Eigen::MatrixXd BMat(array->GetTargets().size(), 3);
            for (int i = 0; i < static_cast<int>(array->GetTargets().size()); ++i) {
                const auto &tar = array->GetTargets().at(i);
                lVec(i) = tar->GetRadialVelocity() * tar->GetTargetXYZ().norm();
                BMat.block<1, 3>(i, 0) = -tar->GetTargetXYZ().transpose();
            }
            Eigen::Vector3d xVec = (BMat.transpose() * BMat).inverse() * BMat.transpose() * lVec;
            return xVec;
        }

        static void Learn() {
            ns_ctraj::Viewer viewer;
            auto trajBtoB0 = ns_ctraj::SimuEightShapeMotion<4>(5.0, 4.0, 0.5);
            auto BtoB0 = *trajBtoB0.GetTrajectory();
            {
                for (const auto &item: BtoB0.Sampling(0.1)) {
                    double t = item.timeStamp;
                    viewer.AddEntity(ns_viewer::Camera::Create(ns_viewer::Posed(
                            BtoB0.Pose(t).so3().matrix(), BtoB0.LinearVeloInRef(t)
                    ).cast<float>()));
                }
            }
            auto SE3_RtoB = RadarModelLearner::GeneratePoseBias();
            const auto &[mesVec, targets] = SimuRadarData(BtoB0, SE3_RtoB);
            const std::string radarTopic = "/radar";
            const std::string imuTopic = "/imu";
            // 400 hz
            Configor::DataStream::IMUTopics.insert({imuTopic, ""});
            Configor::DataStream::RadarTopics.insert({radarTopic, ""});
            auto calibParam = CalibParamManager::Create();
            calibParam->InitializeParametersFromConfigor();
            auto imuData = BtoB0.ComputeIMUMeasurement(calibParam->GRAVITY, 1 / 400.0);

            calibParam->EXTRI.SO3_RjToBc.at(radarTopic) = SE3_RtoB.so3();
            calibParam->EXTRI.POS_RjInBc.at(radarTopic) = SE3_RtoB.translation();
            calibParam->Save("/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/debug/param.json");

            calibParam->EXTRI.SO3_RjToBc.at(radarTopic) = Sophus::SO3d();
            calibParam->EXTRI.POS_RjInBc.at(radarTopic) = Eigen::Vector3d::Zero();

            auto trajectory = ns_ctraj::Trajectory<4>::Create(0.1, BtoB0.MinTime(), BtoB0.MaxTime());
            {
                // ------------------------------
                // step1: fit rotational b-spline
                // ------------------------------
                spdlog::info("fit rotational b-spline");

                auto estimator = SplineEstimator::Create(trajectory, calibParam);

                for (const auto &item: imuData) {
                    // this factor is fine!!!
                    estimator->AddIMUGyroMeasurement(
                            item, imuTopic, OptOption::OPT_SO3 | OptOption::OPT_SO3_BiToBc, 1.0
                    );
                }
                estimator->AddSO3Centralization(calibParam->EXTRI.SO3_BiToBc_AddressVec(), 1E4,
                                                OptOption::OPT_SO3_BiToBc);
                estimator->FixFirSO3ControlPoint();

                // for (const auto &item: BtoB0.Sampling(0.001)) {
                //     ns_ctraj::Posed pose = item;
                //     pose.t = BtoB0.LinearVeloInRef(item.timeStamp);
                //     estimator->AddPOSMeasurement(pose, ns_ctraj::OptimizationOption::OPT_POS, 1.0);
                //     estimator->AddSO3Measurement(pose, ns_ctraj::OptimizationOption::OPT_SO3, 1.0);
                // }

                estimator->Solve();
                trajectory->Visualization(viewer);

                // verify so3 b-spline fitting
                // for (const auto &item: imuData) {
                //     double t = item->GetTimestamp();
                //
                //     // from estimated so3 -b-spline
                //     Eigen::Vector3d v1 =
                //             (trajectory->Pose(t).so3() * calibParam->EXTRI.SO3_BiToBc.at(imuTopic)).inverse() *
                //             trajectory->AngularVeloInRef(t);
                //     // from ground truth
                //     Eigen::Vector3d v2 = item->GetGyro().transpose();
                //
                //     std::cout << v2.transpose() << std::endl;
                //     std::cout << v1.transpose() << std::endl << std::endl;
                // }
            }

            // set wrong initial value
            calibParam->GRAVITY = SE3_RtoB.so3() * calibParam->GRAVITY;
            calibParam->ShowParamStatus();

            {
                // -------------------------
                // test split initialization
                // -------------------------
                // for (const auto &array: mesVec) {
                //     double t = array->GetTimestamp();
                //     std::cout << t << std::endl;
                //
                //     Eigen::Vector3d v1 = RadarVelocityFromStaticTargetArray(array);
                //     Eigen::Vector3d v2 = SE3_RtoB.so3().inverse() * trajectory->Pose(t).so3().inverse() * (
                //             -Sophus::SO3d::hat(trajectory->Pose(t).so3() * SE3_RtoB.translation()) *
                //             trajectory->AngularVeloInRef(t) + trajectory->Pose(t).translation()
                //     );
                //     std::cout << v1.transpose() << std::endl;
                //     std::cout << v2.transpose() << std::endl << std::endl;
                //
                //     Eigen::Vector3d v3 = trajectory->Pose(t).translation();
                //     Eigen::Vector3d v4 = trajectory->Pose(t).so3() * SE3_RtoB.so3() * v1 +
                //                          Sophus::SO3d::hat(trajectory->Pose(t).so3() * SE3_RtoB.translation()) *
                //                          trajectory->AngularVeloInRef(t);
                //     std::cout << v3.transpose() << std::endl;
                //     std::cout << v4.transpose() << std::endl << std::endl << std::endl;
                // }
                auto estimator = SplineEstimator::Create(trajectory, calibParam);
                for (int i = 0; i < static_cast<int>(mesVec.size()) - 1; ++i) {
                    int j = i + 1;
                    const auto &arrayI = mesVec.at(i);
                    const auto &arrayJ = mesVec.at(j);
                    double ti = arrayI->GetTimestamp(), tj = arrayJ->GetTimestamp();
                    Eigen::Vector3d vI = RadarVelocityFromStaticTargetArray(arrayI);
                    Eigen::Vector3d vJ = RadarVelocityFromStaticTargetArray(arrayJ);
                    // this factor is fine!!!
                    estimator->AddDiscreteVelPreIntegration(
                            imuData, imuTopic, radarTopic, ti, tj, vI, vJ,
                            OptOption::OPT_GRAVITY | OptOption::OPT_POS_BiInBc |
                            OptOption::OPT_SO3_RjToBc | OptOption::OPT_POS_RjInBc, 1.0
                    );
                }
                estimator->AddPOSCentralization(
                        calibParam->EXTRI.POS_BiInBc_AddressVec(), 1E4, OptOption::OPT_POS_BiInBc
                );
                estimator->Solve();
                calibParam->ShowParamStatus();
            }
            {
                // --------------------------------
                // step2: extrinsics initialization
                // --------------------------------
                spdlog::info("extrinsics initialization");

                auto estimator = SplineEstimator::Create(trajectory, calibParam);
                for (const auto &item: mesVec) {
                    for (const auto &item2: item->GetTargets()) {
                        // this factor is ok!
                        estimator->AddRadarMeasurement(
                                item2, radarTopic,
                                OptOption::OPT_VEL | OptOption::OPT_SO3_RjToBc | OptOption::OPT_POS_RjInBc, 1.0
                        );
                    }
                }
                for (double t = BtoB0.MinTime(); t < BtoB0.MaxTime();) {
                    // this factor is ok!
                    estimator->AddVelPreIntegration(
                            imuData, imuTopic, t, t + 0.1,
                            OptOption::OPT_VEL | OptOption::OPT_GRAVITY | OptOption::OPT_POS_BiInBc, 10.0
                    );
                    t += 0.1;
                }
                estimator->AddPOSCentralization(calibParam->EXTRI.POS_BiInBc_AddressVec(), 1E4,
                                                OptOption::OPT_POS_BiInBc);
                estimator->Solve();
                trajectory->Visualization(viewer);
            }
            {
                // -------------------------
                // step3: batch optimization
                // -------------------------
                Configor::Prior::TimeOffsetPadding = 0.01;

                spdlog::info("batch optimization");
                auto estimator = SplineEstimator::Create(trajectory, calibParam);
                for (const auto &item: imuData) {
                    estimator->AddIMUGyroMeasurement(
                            item, imuTopic, OptOption::OPT_SO3 | OptOption::OPT_SO3_BiToBc |
                                            OptOption::OPT_TIME_OFFSET_BiToBc |
                                            OptOption::OPT_GYRO_BIAS | OptOption::OPT_GYRO_MAP_COEFF |
                                            OptOption::OPT_SO3_AtoG, 1.0
                    );
                    estimator->AddIMUAcceMeasurement(
                            item, imuTopic, OptOption::OPT_SO3 | OptOption::OPT_VEL |
                                            OptOption::OPT_GRAVITY |
                                            OptOption::OPT_SO3_BiToBc | OptOption::OPT_POS_BiInBc |
                                            OptOption::OPT_TIME_OFFSET_BiToBc |
                                            OptOption::OPT_ACCE_BIAS | OptOption::OPT_ACCE_MAP_COEFF, 1.0
                    );
                }
                for (const auto &item: mesVec) {
                    for (const auto &item2: item->GetTargets()) {
                        estimator->AddRadarMeasurement(
                                item2, radarTopic, OptOption::OPT_SO3 | OptOption::OPT_VEL |
                                                   OptOption::OPT_SO3_RjToBc | OptOption::OPT_POS_RjInBc |
                                                   OptOption::OPT_TIME_OFFSET_RjToBc, 1.0
                        );
                    }
                }
                estimator->FixFirSO3ControlPoint();
                estimator->AddSO3Centralization(
                        calibParam->EXTRI.SO3_BiToBc_AddressVec(), 1E4, OptOption::OPT_SO3_BiToBc
                );
                estimator->AddPOSCentralization(
                        calibParam->EXTRI.POS_BiInBc_AddressVec(), 1E4, OptOption::OPT_POS_BiInBc
                );
                estimator->AddTimeOffsetCentralization(
                        calibParam->TEMPORAL.TIME_OFFSET_BiToBc_AddressVec(), 1E4, OptOption::OPT_TIME_OFFSET_BiToBc
                );
                estimator->Solve();
            }
            calibParam->ShowParamStatus();
            std::cout << SE3_RtoB.so3().matrix().eulerAngles(0, 1, 2).transpose() * CalibParamManager::RAD_TO_DEG
                      << std::endl;
            std::cout << SE3_RtoB.translation().transpose() << std::endl;

            trajectory->Visualization(viewer, 0.01);
            for (const auto &item: targets) {
                viewer.AddEntity(ns_viewer::Landmark::Create(item.cast<float>()));
            }
            viewer.RunInSingleThread();

            // save
//            IMUFrame::SaveFramesToDisk(
//                    "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/debug/imu_data.json", imuData
//            );
//            RadarTargetArray::SaveTargetArraysToDisk(
//                    "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/debug/radar_data.json", mesVec
//            );
//            BtoB0.Save("/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/debug/traj.json");
        }
    };
}

#endif //RIS_CALIB_RADAR_MODEL_HPP
