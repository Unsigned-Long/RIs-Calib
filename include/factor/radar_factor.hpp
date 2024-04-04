// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_RADAR_FACTOR_HPP
#define RIS_CALIB_RADAR_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/radar.h"

namespace ns_ris {
    template<int Order>
    struct RadarFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;
        RadarTarget::Ptr _radarFrame{};

        double _dtInv;
        double _weight;
    public:
        explicit RadarFactor(ns_ctraj::SplineMeta<Order> splineMeta, RadarTarget::Ptr radarFrame, double weight)
                : _splineMeta(std::move(splineMeta)), _radarFrame(std::move(radarFrame)),
                  _dtInv(1.0 / _splineMeta.segments.front().dt), _weight(weight) {}

        static auto Create(ns_ctraj::SplineMeta<Order> splineMeta, const RadarTarget::Ptr &radarFrame, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<RadarFactor>(
                    new RadarFactor(splineMeta, radarFrame, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(RadarFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3 | ... | SO3 | VEL | ... | VEL | SO3_RtoB | POS_RinB | TIME_OFFSET_RtoB ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);

            std::size_t SO3_OFFSET, VEL_OFFSET;
            std::size_t SO3_RtoB_OFFSET = 2 * _splineMeta.NumParameters();
            std::size_t POS_RinB_OFFSET = SO3_RtoB_OFFSET + 1;
            std::size_t TIME_OFFSET_RtoB_OFFSET = POS_RinB_OFFSET + 1;

            // get value
            Eigen::Map<const Sophus::SO3<T>> SO3_RtoB(sKnots[SO3_RtoB_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> POS_RinB(sKnots[POS_RinB_OFFSET]);
            T TIME_OFFSET_RtoB = sKnots[TIME_OFFSET_RtoB_OFFSET][0];

            auto timeByIMU = _radarFrame->GetTimestamp() + TIME_OFFSET_RtoB;

            // calculate the so3 and pos offset
            std::pair<std::size_t, T> pointIU;
            _splineMeta.template ComputeSplineIndex(timeByIMU, pointIU.first, pointIU.second);

            SO3_OFFSET = pointIU.first;
            VEL_OFFSET = SO3_OFFSET + _splineMeta.NumParameters();

            // query
            Sophus::SO3<T> SO3_BtoB0;
            Eigen::Vector3<T> ANG_VEL_BtoB0InB;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template EvaluateLie(
                    sKnots + SO3_OFFSET, pointIU.second, _dtInv, &SO3_BtoB0, &ANG_VEL_BtoB0InB
            );
            Eigen::Vector3<T> ANG_VEL_BtoB0InB0 = SO3_BtoB0 * ANG_VEL_BtoB0InB;

            /**
             * @attention: current R^3 trajectory is the velocity b-spline, whose
             * first zero derivative is the linear velocity, not the first order derivative!!!
             */
            Eigen::Vector3<T> LIN_VEL_BInB0;
            ns_ctraj::CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
                    sKnots + VEL_OFFSET, pointIU.second, _dtInv, &LIN_VEL_BInB0
            );

            Eigen::Vector3<T> tarInR = _radarFrame->GetTargetXYZ().cast<T>();
            Eigen::Vector1<T> v1 =
                    -tarInR.transpose() * SO3_RtoB.matrix().transpose() * SO3_BtoB0.matrix().transpose() * (
                            -Sophus::SO3<T>::hat(SO3_BtoB0 * POS_RinB) * ANG_VEL_BtoB0InB0 + LIN_VEL_BInB0
                    );

            T v2 = static_cast<T>(_radarFrame->GetRadialVelocity());

            residuals.template block<1, 1>(0, 0) =
                    T(_weight) * Eigen::Vector1<T>(_radarFrame->GetInvRange() * v1(0) - v2);

            return true;
        }
    };

}


#endif //RIS_CALIB_RADAR_FACTOR_HPP
