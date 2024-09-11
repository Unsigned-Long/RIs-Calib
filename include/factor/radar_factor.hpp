// RIs-Calib: An Open-Source Spatiotemporal Calibrator for Multiple 3D Radars and IMUs Based on Continuous-Time Estimation
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/RIs-Calib.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
