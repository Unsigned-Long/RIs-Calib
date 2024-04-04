// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "calib/spline_estimator.h"
#include <utility>
#include "factor/imu_acce_factor.hpp"
#include "factor/imu_gyro_factor.hpp"
#include "factor/radar_factor.hpp"
#include "factor/pre_integration_factor.hpp"
#include "factor/centralization_factor.hpp"

namespace ns_ris {

    SplineEstimator::SplineEstimator(const Trajectory::Ptr &trajectory, CalibParamManager::Ptr calibParam)
            : Parent(trajectory), _calibParam(std::move(calibParam)) {}

    SplineEstimator::Ptr
    SplineEstimator::Create(const Trajectory::Ptr &trajectory, const CalibParamManager::Ptr &calibParam) {
        return std::make_shared<SplineEstimator>(trajectory, calibParam);
    }

    LMEquation SplineEstimator::Evaluate(const std::map<const double *, std::string> &targetParamsInfoMapRaw) {
        std::map<const double *, std::string> targetParamsInfoMap;
        for (const auto &[key, value]: targetParamsInfoMapRaw) {
            if (this->HasParameterBlock(key)) { targetParamsInfoMap.insert({key, value}); }
        }

        std::size_t rows = 0, cols = 0;
        // param address, param dime, param start column
        std::map<const double *, std::pair<std::size_t, std::size_t>> paramDimeInfo;
        for (auto &factor: _factors) {
            factor.Evaluate(targetParamsInfoMap);
            // record size info
            rows += factor.GetResiduals().size();
            for (const auto &[pAddress, pJacobian]: factor.GetJacobians()) {
                // using map assign operator, if the 'pAddress' not exists, a new element will be created,
                // otherwise, the value of the key will be updated
                paramDimeInfo[pAddress] = {pJacobian.cols(), 0};
            }
        }

        // desc, address, handle the param ordered by the desc
        std::map<std::string, const double *> invTargetParamsInfoMap;
        for (const auto &[key, value]: targetParamsInfoMap) { invTargetParamsInfoMap.insert({value, key}); }

        // desc, dime
        std::vector<std::pair<std::string, std::size_t>> paramDesc;
        for (auto &[desc, pAddress]: invTargetParamsInfoMap) {
            auto &column = paramDimeInfo.at(pAddress);
            column.second = cols;
            cols += column.first;
            paramDesc.emplace_back(desc, column.first);
        }

        Eigen::MatrixXd jMat(rows, cols), hMat(cols, cols);
        Eigen::VectorXd rVec(rows), bVec(cols);
        jMat.setZero(), rVec.setZero();

        // factor type id, residuals
        std::map<std::size_t, Eigen::aligned_vector<Eigen::VectorXd>> residualsMap;
        // organize the jMat and rVec
        int cr = 0;
        for (const auto &factor: _factors) {
            const auto &jacobians = factor.GetJacobians();
            const auto &residuals = factor.GetResiduals();
            // residuals vector
            rVec.block(cr, 0, residuals.rows(), 1) = residuals;

            // jacobians matrix
            for (const auto &[pAddress, pJacobian]: jacobians) {
                jMat.block(
                        cr, static_cast<int>(paramDimeInfo.find(pAddress)->second.second),
                        pJacobian.rows(), pJacobian.cols()
                ) = pJacobian;
            }

            // draw error
            residualsMap[factor.GetCostFunctorHashCode()].push_back(residuals);

            cr += static_cast<int>(residuals.rows());
        }
        hMat = jMat.transpose() * jMat;
        bVec = -jMat.transpose() * rVec;

        return {hMat, bVec, paramDesc, residualsMap};
    }

    LMEquation SplineEstimator::Evaluate(
            const std::initializer_list<std::map<const double *, std::string>> &targetParamsInfoMaps) {

        std::map<const double *, std::string> targetParamsInfoMap;
        int count = static_cast<int>(targetParamsInfoMaps.size()), idx = 0;
        for (const auto &item: targetParamsInfoMaps) {
            std::string identify = fmt::format("P{:0" + std::to_string((int) log10(count) + 1) + "}:", idx);
            for (auto p: item) {
                p.second = identify + p.second;
                targetParamsInfoMap.insert(p);
            }
            ++idx;
        }

        return Evaluate(targetParamsInfoMap);
    }

    // -------
    // factors
    // -------
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL | ACCE_BIAS | ACCE_MAP_COEFF | GRAVITY |
     *   SO3_BiToBc | POS_BiInBc | TIME_OFFSET_BiToBc ]
     */
    void SplineEstimator::AddIMUAcceMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                                int option, double acceWeight) {
        // find the affected control points
        SplineMeta splineMeta;

        // different relative control points finding [single vs. range]
        if (OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_BiToBc, option)) {
            double frameIMUTimeMin = imuFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
            double frameIMUTimeMax = imuFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
            // invalid time stamp
            if (!_trajectory->TimeStampInRange(frameIMUTimeMin) ||
                !_trajectory->TimeStampInRange(frameIMUTimeMax)) {
                return;
            }
            _trajectory->CalculateSplineMeta({{frameIMUTimeMin, frameIMUTimeMax}}, splineMeta);
        } else {
            double frameIMUTime = imuFrame->GetTimestamp() + _calibParam->TEMPORAL.TIME_OFFSET_BiToBc.at(topic);

            // check point time stamp
            if (!_trajectory->TimeStampInRange(frameIMUTime)) {
                return;
            }

            _trajectory->CalculateSplineMeta({{frameIMUTime, frameIMUTime}}, splineMeta);
        }

        // create a cost function
        auto costFunc = IMUAcceFactor<Configor::Prior::SplineOrder>::Create(splineMeta, imuFrame, acceWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < static_cast<int>(splineMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // pos knots param block [each has three sub params]
        for (int i = 0; i < static_cast<int>(splineMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        // ACCE_BIAS
        costFunc->AddParameterBlock(3);
        // ACCE_MAP_COEFF
        costFunc->AddParameterBlock(6);
        // GRAVITY
        costFunc->AddParameterBlock(3);
        // SO3_BiToBc
        costFunc->AddParameterBlock(4);
        // POS_BiInBc
        costFunc->AddParameterBlock(3);
        // TIME_OFFSET_BiToBc
        costFunc->AddParameterBlock(1);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS, splineMeta,
                !OptOption::IsOptionWith(OptOption::OPT_SO3, option)
        );

        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS, splineMeta,
                !OptOption::IsOptionWith(OptOption::OPT_VEL, option)
        );

        // ACCE_BIAS
        auto acceBias = _calibParam->INTRI.IMU.at(topic).ACCE.BIAS.data();
        paramBlockVec.push_back(acceBias);
        // ACCE_MAP_COEFF
        auto aceMapCoeff = _calibParam->INTRI.IMU.at(topic).ACCE.MAP_COEFF.data();
        paramBlockVec.push_back(aceMapCoeff);
        // GRAVITY
        auto gravity = _calibParam->GRAVITY.data();
        paramBlockVec.push_back(gravity);
        // SO3_BiToBc
        auto SO3_BiToBc = _calibParam->EXTRI.SO3_BiToBc.at(topic).data();
        paramBlockVec.push_back(SO3_BiToBc);
        // POS_BiInBc
        auto POS_BiInBc = _calibParam->EXTRI.POS_BiInBc.at(topic).data();
        paramBlockVec.push_back(POS_BiInBc);
        // TIME_OFFSET_BiToBc
        auto TIME_OFFSET_BiToBc = &_calibParam->TEMPORAL.TIME_OFFSET_BiToBc.at(topic);
        paramBlockVec.push_back(TIME_OFFSET_BiToBc);

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity, S2_MANIFOLD.get());
        this->SetManifold(SO3_BiToBc, QUATER_MANIFOLD.get());

        if (!OptOption::IsOptionWith(OptOption::OPT_ACCE_BIAS, option)) {
            this->SetParameterBlockConstant(acceBias);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_ACCE_MAP_COEFF, option)) {
            this->SetParameterBlockConstant(aceMapCoeff);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_SO3_BiToBc, option)) {
            this->SetParameterBlockConstant(SO3_BiToBc);
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_POS_BiInBc, option)) {
            this->SetParameterBlockConstant(POS_BiInBc);
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_BiToBc, option)) {
            this->SetParameterBlockConstant(TIME_OFFSET_BiToBc);
        } else {
            // set bound
            this->SetParameterLowerBound(TIME_OFFSET_BiToBc, 0, -Configor::Prior::TimeOffsetPadding);
            this->SetParameterUpperBound(TIME_OFFSET_BiToBc, 0, Configor::Prior::TimeOffsetPadding);
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF | SO3_AtoG | SO3_BiToBc | TIME_OFFSET_BiToBc ]
     */
    void SplineEstimator::AddIMUGyroMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                                int option, double gyroWeight) {
        // find the affected control points
        SplineMeta splineMeta;

        // different relative control points finding [single vs. range]
        if (OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_BiToBc, option)) {
            double frameIMUTimeMin = imuFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
            double frameIMUTimeMax = imuFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
            // invalid time stamp
            if (!_trajectory->TimeStampInRange(frameIMUTimeMin) ||
                !_trajectory->TimeStampInRange(frameIMUTimeMax)) {
                return;
            }
            _trajectory->CalculateSplineMeta({{frameIMUTimeMin, frameIMUTimeMax}}, splineMeta);
        } else {
            double frameIMUTime = imuFrame->GetTimestamp() + _calibParam->TEMPORAL.TIME_OFFSET_BiToBc.at(topic);

            // check point time stamp
            if (!_trajectory->TimeStampInRange(frameIMUTime)) {
                return;
            }

            _trajectory->CalculateSplineMeta({{frameIMUTime, frameIMUTime}}, splineMeta);
        }

        // create a cost function
        auto costFunc = IMUGyroFactor<Configor::Prior::SplineOrder>::Create(splineMeta, imuFrame, gyroWeight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < static_cast<int>(splineMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }

        // GYRO gyroBias
        costFunc->AddParameterBlock(3);
        // GYRO map coeff
        costFunc->AddParameterBlock(6);
        // SO3_AtoG
        costFunc->AddParameterBlock(4);
        // SO3_BiToBc
        costFunc->AddParameterBlock(4);
        // TIME_OFFSET_BiToBc
        costFunc->AddParameterBlock(1);

        // set Residuals
        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(
                paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS,
                splineMeta, !OptOption::IsOptionWith(OptOption::OPT_SO3, option)
        );
        // GYRO gyroBias
        auto gyroBias = _calibParam->INTRI.IMU.at(topic).GYRO.BIAS.data();
        paramBlockVec.push_back(gyroBias);
        // GYRO map coeff
        auto gyroMapCoeff = _calibParam->INTRI.IMU.at(topic).GYRO.MAP_COEFF.data();
        paramBlockVec.push_back(gyroMapCoeff);
        // SO3_AtoG
        auto SO3_AtoG = _calibParam->INTRI.IMU.at(topic).SO3_AtoG.data();
        paramBlockVec.push_back(SO3_AtoG);
        // SO3_BiToBc
        auto SO3_BiToBc = _calibParam->EXTRI.SO3_BiToBc.at(topic).data();
        paramBlockVec.push_back(SO3_BiToBc);
        // TIME_OFFSET_BiToBc
        auto TIME_OFFSET_BiToBc = &_calibParam->TEMPORAL.TIME_OFFSET_BiToBc.at(topic);
        paramBlockVec.push_back(TIME_OFFSET_BiToBc);


        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);

        this->SetManifold(SO3_AtoG, QUATER_MANIFOLD.get());
        this->SetManifold(SO3_BiToBc, QUATER_MANIFOLD.get());

        if (!OptOption::IsOptionWith(OptOption::OPT_GYRO_BIAS, option)) {
            this->SetParameterBlockConstant(gyroBias);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_GYRO_MAP_COEFF, option)) {
            this->SetParameterBlockConstant(gyroMapCoeff);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_SO3_AtoG, option)) {
            this->SetParameterBlockConstant(SO3_AtoG);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_SO3_BiToBc, option)) {
            this->SetParameterBlockConstant(SO3_BiToBc);
        }

        if (!OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_BiToBc, option)) {
            this->SetParameterBlockConstant(TIME_OFFSET_BiToBc);
        } else {
            // set bound
            this->SetParameterLowerBound(TIME_OFFSET_BiToBc, 0, -Configor::Prior::TimeOffsetPadding);
            this->SetParameterUpperBound(TIME_OFFSET_BiToBc, 0, Configor::Prior::TimeOffsetPadding);
        }
    }

    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL | SO3_RjToBc | POS_RjInBc | TIME_OFFSET_RjToBc ]
     */
    void SplineEstimator::AddRadarMeasurement(const RadarTarget::Ptr &radarFrame, const std::string &topic,
                                              int option, double weight) {
        SplineMeta splineMeta;

        // different relative control points finding [single vs. range]
        if (OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_RjToBc, option)) {
            double tMin = radarFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
            double tMax = radarFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
            // invalid time stamp
            if (!_trajectory->TimeStampInRange(tMin) || !_trajectory->TimeStampInRange(tMax)) {
                return;
            }
            _trajectory->CalculateSplineMeta({{tMin, tMax}}, splineMeta);
        } else {
            double t = radarFrame->GetTimestamp() + _calibParam->TEMPORAL.TIME_OFFSET_RjToBc.at(topic);

            // check point time stamp
            if (!_trajectory->TimeStampInRange(t)) { return; }

            _trajectory->CalculateSplineMeta({{t, t}}, splineMeta);
        }

        // create a cost function
        auto costFunc = RadarFactor<Configor::Prior::SplineOrder>::Create(splineMeta, radarFrame, weight);

        // so3 knots param block [each has four sub params]
        for (int i = 0; i < static_cast<int>(splineMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(4);
        }
        // vel knots param block [each has three sub params]
        for (int i = 0; i < static_cast<int>(splineMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }
        costFunc->AddParameterBlock(4); // SO3_RtoB
        costFunc->AddParameterBlock(3); // POS_RinB
        costFunc->AddParameterBlock(1); // TIME_OFFSET_RtoB

        // the Residual
        costFunc->SetNumResiduals(1);

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // so3 knots param block
        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::SO3_KNOTS,
                          splineMeta, !OptOption::IsOptionWith(OptOption::OPT_SO3, option));

        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS,
                          splineMeta, !OptOption::IsOptionWith(OptOption::OPT_VEL, option));

        paramBlockVec.push_back(_calibParam->EXTRI.SO3_RjToBc.at(topic).data());
        paramBlockVec.push_back(_calibParam->EXTRI.POS_RjInBc.at(topic).data());
        paramBlockVec.push_back(&_calibParam->TEMPORAL.TIME_OFFSET_RjToBc.at(topic));

        // pass to problem
        this->AddResidualBlockToProblem(
                costFunc, new ceres::CauchyLoss(Configor::Prior::CauchyLossForRadarFactor * weight), paramBlockVec
        );
        // remove dynamic targets (outliers)
        // this->AddResidualBlockToProblem(costFunc, new ceres::HuberLoss(weight * weight * 0.125), paramBlockVec);

        this->SetManifold(_calibParam->EXTRI.SO3_RjToBc.at(topic).data(), QUATER_MANIFOLD.get());

        // lock param or not
        if (!OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_RjToBc, option)) {
            this->SetParameterBlockConstant(&_calibParam->TEMPORAL.TIME_OFFSET_RjToBc.at(topic));
        } else {
            // set bound
            this->SetParameterLowerBound(
                    &_calibParam->TEMPORAL.TIME_OFFSET_RjToBc.at(topic), 0, -Configor::Prior::TimeOffsetPadding
            );
            this->SetParameterUpperBound(
                    &_calibParam->TEMPORAL.TIME_OFFSET_RjToBc.at(topic), 0, Configor::Prior::TimeOffsetPadding
            );
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_SO3_RjToBc, option)) {
            this->SetParameterBlockConstant(_calibParam->EXTRI.SO3_RjToBc.at(topic).data());
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_POS_RjInBc, option)) {
            this->SetParameterBlockConstant(_calibParam->EXTRI.POS_RjInBc.at(topic).data());
        }
    }

    /**
     * param blocks:
     * [ VEL | ... | VEL | GRAVITY | POS_BiInBc ]
     */
    void
    SplineEstimator::AddVelPreIntegration(const std::vector<IMUFrame::Ptr> &imuMes, const std::string &imuTopic,
                                          double ti, double tj, int option, double weight) {
        SplineMeta splineMeta;

        if (!_trajectory->TimeStampInRange(ti) || !_trajectory->TimeStampInRange(tj)) { return; }

        // check point time stamp
        _trajectory->CalculateSplineMeta({{ti, ti},
                                          {tj, tj}}, splineMeta);


        auto [sIter, eIter] = ExtractRange(imuMes, ti, tj);
        std::vector<std::pair<double, Eigen::Matrix3d>> reorganizedSubData1;
        std::vector<std::pair<double, Eigen::Vector3d>> reorganizedSubData2;
        for (auto iter = sIter; iter != eIter; ++iter) {
            const auto &frame = *iter;
            double t = frame->GetTimestamp();

            Eigen::Matrix3d m1 = Sophus::SO3d::hat(_trajectory->AngularAcceInRef(t)).matrix();
            Eigen::Matrix3d m2 = Sophus::SO3d::hat(_trajectory->AngularVeloInRef(t)).matrix();
            Eigen::Matrix3d m3 = (m1 + m2 * m2) * _trajectory->Pose(t).so3().matrix();
            reorganizedSubData1.emplace_back(t, m3);

            Eigen::Vector3d v1 =
                    _trajectory->Pose(t).so3() * _calibParam->EXTRI.SO3_BiToBc.at(imuTopic) * frame->GetAcce();
            reorganizedSubData2.emplace_back(t, v1);
        }

        auto costFunc = PreIntegrationFactor<Configor::Prior::SplineOrder>::Create(
                splineMeta, ti, tj, TrapIntegrationOnce(reorganizedSubData1),
                TrapIntegrationOnce(reorganizedSubData2), weight
        );

        // organize the param block vector
        std::vector<double *> paramBlockVec;

        // vel knots param block [each has three sub params]
        for (int i = 0; i < static_cast<int>(splineMeta.NumParameters()); ++i) {
            costFunc->AddParameterBlock(3);
        }

        // GRAVITY
        costFunc->AddParameterBlock(3);
        // POS_BiInBc
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        AddCtrlPointsData(paramBlockVec, AddCtrlPointsDataFlag::POS_KNOTS,
                          splineMeta, !OptOption::IsOptionWith(OptOption::OPT_VEL, option));
        // GRAVITY
        auto gravity = _calibParam->GRAVITY.data();
        paramBlockVec.push_back(gravity);
        // POS_BiInBc
        auto POS_BiInBc = _calibParam->EXTRI.POS_BiInBc.at(imuTopic).data();
        paramBlockVec.push_back(POS_BiInBc);

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity, S2_MANIFOLD.get());

        if (!OptOption::IsOptionWith(OptOption::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity);
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_POS_BiInBc, option)) {
            this->SetParameterBlockConstant(POS_BiInBc);
        }
    }

    /**
     * [ SO3_BiToBc ]
     */
    void SplineEstimator::AddSO3Centralization(const std::vector<Sophus::SO3d *> &SO3_StoRef,
                                               double weight, std::uint32_t option) {
        auto costFunc = SO3CentralizationFactor::Create(SO3_StoRef.size(), weight);

        std::vector<double *> paramBlockVec;

        for (auto &item: SO3_StoRef) {
            // SO3_StoRef
            costFunc->AddParameterBlock(4);
            paramBlockVec.emplace_back(item->data());
            this->AddParameterBlock(item->data(), 4, QUATER_MANIFOLD.get());
        }
        costFunc->SetNumResiduals(3);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        if (!OptOption::IsOptionWith(OptOption::OPT_SO3_BiToBc, option)) {
            for (auto &item: SO3_StoRef) { this->SetParameterBlockConstant(item->data()); }
        }

    }

    /**
     * [ POS_BiInBc ]
     */
    void SplineEstimator::AddPOSCentralization(const std::vector<Eigen::Vector3d *> &POS_SinRef,
                                               double weight, std::uint32_t option) {
        auto costFunc = POSCentralizationFactor::Create(POS_SinRef.size(), weight);

        std::vector<double *> paramBlockVec;

        for (auto &item: POS_SinRef) {
            // POS_SinRef
            costFunc->AddParameterBlock(3);
            paramBlockVec.emplace_back(item->data());
        }
        costFunc->SetNumResiduals(3);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        if (!OptOption::IsOptionWith(OptOption::OPT_POS_BiInBc, option)) {
            for (auto &item: POS_SinRef) { this->SetParameterBlockConstant(item->data()); }
        }
    }

    /**
     * [ TIME_OFFSET_BiToBc ]
     */
    void SplineEstimator::AddTimeOffsetCentralization(const std::vector<double *> &TIME_OFFSET_StoRef,
                                                      double weight, std::uint32_t option) {
        auto costFunc = TimeOffsetCentralizationFactor::Create(TIME_OFFSET_StoRef.size(), weight);

        std::vector<double *> paramBlockVec;

        for (auto &item: TIME_OFFSET_StoRef) {
            // TIME_OFFSET_StoRef
            costFunc->AddParameterBlock(1);
            paramBlockVec.emplace_back(item);
        }
        costFunc->SetNumResiduals(1);

        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);

        if (!OptOption::IsOptionWith(OptOption::OPT_TIME_OFFSET_BiToBc, option)) {
            for (auto &item: TIME_OFFSET_StoRef) { this->SetParameterBlockConstant(item); }
        } else {
            // set bound
            for (auto &item: TIME_OFFSET_StoRef) {
                this->SetParameterLowerBound(item, 0, -Configor::Prior::TimeOffsetPadding);
                this->SetParameterUpperBound(item, 0, Configor::Prior::TimeOffsetPadding);
            }
        }
    }

    /**
     * param blocks:
     * [ GRAVITY | POS_BiInBc | SO3_RjToBc | POS_RjInBc ]
     */
    void SplineEstimator::AddDiscreteVelPreIntegration(const std::vector<IMUFrame::Ptr> &imuMes,
                                                       const std::string &imuTopic, const std::string &radarTopic,
                                                       double ti, double tj,
                                                       const Eigen::Vector3d &radarVelI,
                                                       const Eigen::Vector3d &radarVelJ,
                                                       int option, double weight) {
        auto [sIter, eIter] = ExtractRange(imuMes, ti, tj);
        std::vector<std::pair<double, Eigen::Matrix3d>> reorganizedSubData1;
        std::vector<std::pair<double, Eigen::Vector3d>> reorganizedSubData2;
        for (auto iter = sIter; iter != eIter; ++iter) {
            const auto &frame = *iter;
            double t = frame->GetTimestamp();

            Eigen::Matrix3d m1 = Sophus::SO3d::hat(_trajectory->AngularAcceInRef(t)).matrix();
            Eigen::Matrix3d m2 = Sophus::SO3d::hat(_trajectory->AngularVeloInRef(t)).matrix();
            Eigen::Matrix3d m3 = (m1 + m2 * m2) * _trajectory->Pose(t).so3().matrix();
            reorganizedSubData1.emplace_back(t, m3);

            Eigen::Vector3d v1 =
                    _trajectory->Pose(t).so3() * _calibParam->EXTRI.SO3_BiToBc.at(imuTopic) * frame->GetAcce();
            reorganizedSubData2.emplace_back(t, v1);
        }

        auto costFunc = DiscretePreIntegrationFactor<Configor::Prior::SplineOrder>::Create(
                tj - ti, TrapIntegrationOnce(reorganizedSubData1),
                TrapIntegrationOnce(reorganizedSubData2), radarVelI, radarVelJ, _trajectory->AngularVeloInRef(ti),
                _trajectory->AngularVeloInRef(tj), _trajectory->Pose(ti).so3(), _trajectory->Pose(tj).so3(), weight
        );

        // GRAVITY
        costFunc->AddParameterBlock(3);
        // POS_BiInBc
        costFunc->AddParameterBlock(3);
        // SO3_RjToBc
        costFunc->AddParameterBlock(4);
        // POS_RjInBc
        costFunc->AddParameterBlock(3);

        costFunc->SetNumResiduals(3);

        // organize the param block vector
        std::vector<double *> paramBlockVec;
        // GRAVITY
        auto gravity = _calibParam->GRAVITY.data();
        paramBlockVec.push_back(gravity);
        // POS_BiInBc
        auto POS_BiInBc = _calibParam->EXTRI.POS_BiInBc.at(imuTopic).data();
        paramBlockVec.push_back(POS_BiInBc);
        // SO3_RjToBc
        auto SO3_RjToBc = _calibParam->EXTRI.SO3_RjToBc.at(radarTopic).data();
        paramBlockVec.push_back(SO3_RjToBc);
        // POS_RjInBc
        auto POS_RjInBc = _calibParam->EXTRI.POS_RjInBc.at(radarTopic).data();
        paramBlockVec.push_back(POS_RjInBc);

        // pass to problem
        this->AddResidualBlockToProblem(costFunc, nullptr, paramBlockVec);
        this->SetManifold(gravity, S2_MANIFOLD.get());
        this->SetManifold(SO3_RjToBc, QUATER_MANIFOLD.get());

        if (!OptOption::IsOptionWith(OptOption::OPT_GRAVITY, option)) {
            this->SetParameterBlockConstant(gravity);
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_POS_BiInBc, option)) {
            this->SetParameterBlockConstant(POS_BiInBc);
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_SO3_RjToBc, option)) {
            this->SetParameterBlockConstant(SO3_RjToBc);
        }
        if (!OptOption::IsOptionWith(OptOption::OPT_POS_RjInBc, option)) {
            this->SetParameterBlockConstant(POS_RjInBc);
        }
    }

    void SplineEstimator::FixFirSO3ControlPoint() {
        const auto &so3Spline = this->_trajectory->GetSo3Spline();
        for (int i = 0; i < static_cast<int>(so3Spline.GetKnots().size()); ++i) {
            auto data = so3Spline.GetKnot(i).data();
            if (this->HasParameterBlock(data)) {
                this->SetParameterBlockConstant(data);
                break;
            }
        }
    }
}