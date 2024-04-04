// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_IMU_ACCE_FACTOR_HPP
#define RIS_CALIB_IMU_ACCE_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_ris {
    template<int Order>
    struct IMUAcceFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;
        IMUFrame::Ptr _imuFrame{};

        double _dtInv;
        double _acceWeight;
    public:
        explicit IMUAcceFactor(ns_ctraj::SplineMeta<Order> splineMeta, IMUFrame::Ptr imuFrame, double acceWeight)
                : _splineMeta(std::move(splineMeta)), _imuFrame(std::move(imuFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _acceWeight(acceWeight) {}

        static auto
        Create(const ns_ctraj::SplineMeta<Order> &splineMeta, const IMUFrame::Ptr &imuFrame, double acceWeight) {
            return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactor>(
                    new IMUAcceFactor(splineMeta, imuFrame, acceWeight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(IMUAcceFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | VEL | ... | VEL | ACCE_BIAS | ACCE_MAP_COEFF | GRAVITY |
         *   SO3_BiToBc | POS_BiInBc | TIME_OFFSET_BiToBc ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            std::size_t SO3_offset;
            std::size_t POS_offset;

            std::size_t ACCE_BIAS_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t ACCE_MAP_COEFF_OFFSET = ACCE_BIAS_OFFSET + 1;
            std::size_t GRAVITY_OFFSET = ACCE_MAP_COEFF_OFFSET + 1;
            std::size_t SO3_BiToBc_OFFSET = GRAVITY_OFFSET + 1;
            std::size_t POS_BiInBc_OFFSET = SO3_BiToBc_OFFSET + 1;
            std::size_t TIME_OFFSET_BiToBc_OFFSET = POS_BiInBc_OFFSET + 1;

            // get value
            Eigen::Map<const Sophus::SO3<T>> SO3_BiToBc(sKnots[SO3_BiToBc_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> POS_BiInBc(sKnots[POS_BiInBc_OFFSET]);
            T TIME_OFFSET_BiToBc = sKnots[TIME_OFFSET_BiToBc_OFFSET][0];

            auto timeByBc = _imuFrame->GetTimestamp() + TIME_OFFSET_BiToBc;

            // calculate the so3 and pos offset
            std::pair<std::size_t, T> pointIU;
            _splineMeta.template ComputeSplineIndex(timeByBc, pointIU.first, pointIU.second);

            SO3_offset = pointIU.first;
            POS_offset = SO3_offset + _splineMeta.NumParameters();

            Sophus::SO3<T> SO3_BcToBc0;
            Sophus::SO3Tangent<T> SO3_VEL_BcToBc0InBc, SO3_ACCE_BcToBc0InBc;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template EvaluateLie(
                    sKnots + SO3_offset, pointIU.second, _dtInv,
                    &SO3_BcToBc0, &SO3_VEL_BcToBc0InBc, &SO3_ACCE_BcToBc0InBc
            );
            Sophus::SO3Tangent<T> SO3_VEL_BcToBc0InBc0 = SO3_BcToBc0 * SO3_VEL_BcToBc0InBc;
            Sophus::SO3Tangent<T> SO3_ACCE_BcToBc0InBc0 = SO3_BcToBc0 * SO3_ACCE_BcToBc0InBc;

            /**
             * @attention: current R^3 trajectory is the velocity b-spline, whose
             * first order derivative is the linear acceleration, not the second order derivative!!!
             */
            Eigen::Vector3<T> POS_ACCE_BcToBc0InBc0;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 1>(
                    sKnots + POS_offset, pointIU.second, _dtInv, &POS_ACCE_BcToBc0InBc0
            );

            Eigen::Map<const Eigen::Vector3<T>> acceBias(sKnots[ACCE_BIAS_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> gravity(sKnots[GRAVITY_OFFSET]);

            auto acceCoeff = sKnots[ACCE_MAP_COEFF_OFFSET];

            Eigen::Matrix33<T> acceMapMat = Eigen::Matrix33<T>::Zero();

            acceMapMat.diagonal() = Eigen::Map<const Eigen::Vector3<T>>(acceCoeff, 3);
            acceMapMat(0, 1) = *(acceCoeff + 3);
            acceMapMat(0, 2) = *(acceCoeff + 4);
            acceMapMat(1, 2) = *(acceCoeff + 5);

            Sophus::SO3<T> SO3_BiToBc0 = SO3_BcToBc0 * SO3_BiToBc;
            Eigen::Vector3<T> POS_ACCE_BiToBc0InBc0 =
                    -Sophus::SO3<T>::hat(SO3_BcToBc0 * POS_BiInBc) * SO3_ACCE_BcToBc0InBc0 +
                    POS_ACCE_BcToBc0InBc0 - Sophus::SO3<T>::hat(SO3_VEL_BcToBc0InBc0) *
                                            Sophus::SO3<T>::hat(SO3_BcToBc0 * POS_BiInBc) *
                                            SO3_VEL_BcToBc0InBc0;

            Eigen::Vector3<T> accePred =
                    (acceMapMat * (SO3_BiToBc0.inverse() * (POS_ACCE_BiToBc0InBc0 - gravity))).eval() + acceBias;


            Eigen::Vector3<T> acceResiduals = accePred - _imuFrame->GetAcce().template cast<T>();

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            residuals = T(_acceWeight) * acceResiduals;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //RIS_CALIB_IMU_ACCE_FACTOR_HPP
