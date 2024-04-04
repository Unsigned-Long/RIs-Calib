// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_PRE_INTEGRATION_FACTOR_HPP
#define RIS_CALIB_PRE_INTEGRATION_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include <utility>

namespace ns_ris {

    template<int Order>
    struct PreIntegrationFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;

        double _ti;
        double _tj;

        Eigen::Matrix3d DEL_VEL_1;
        Eigen::Vector3d DEL_VEL_2;

        double _dtInv;
        double _weight;

    public:
        explicit PreIntegrationFactor(ns_ctraj::SplineMeta<Order> splineMeta, double ti, double tj,
                                      Eigen::Matrix3d deltaVel1, Eigen::Vector3d deltaVel2, double weight)
                : _splineMeta(std::move(splineMeta)), _ti(ti), _tj(tj), DEL_VEL_1(std::move(deltaVel1)),
                  DEL_VEL_2(std::move(deltaVel2)), _dtInv(1.0 / _splineMeta.segments.front().dt), _weight(weight) {}


        static auto Create(ns_ctraj::SplineMeta<Order> splineMeta, double ti, double tj,
                           const Eigen::Matrix3d &deltaVel1, const Eigen::Vector3d &deltaVel2, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<PreIntegrationFactor>(
                    new PreIntegrationFactor(splineMeta, ti, tj, deltaVel1, deltaVel2, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(PreIntegrationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | ... | VEL | GRAVITY | POS_BiInBc ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            std::size_t V_I_OFFSET, V_J_OFFSET;
            std::size_t GRAVITY_OFFSET = _splineMeta.NumParameters();
            std::size_t POS_BiInBc_OFFSET = GRAVITY_OFFSET + 1;

            Eigen::Map<const Eigen::Vector3<T>> GRAVITY(sKnots[GRAVITY_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> POS_BiInBc(sKnots[POS_BiInBc_OFFSET]);

            std::pair<std::size_t, double> tiIU;
            _splineMeta.ComputeSplineIndex(_ti, tiIU.first, tiIU.second);
            V_I_OFFSET = tiIU.first;
            /**
             * @attention: current R^3 trajectory is the velocity b-spline, whose
             * first zero derivative is the linear velocity, not the first order derivative!!!
             */
            Eigen::Vector3<T> LIN_VEL_I_BcToBc0InBc0;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    sKnots + V_I_OFFSET, tiIU.second, _dtInv, &LIN_VEL_I_BcToBc0InBc0
            );

            std::pair<std::size_t, double> tjIU;
            _splineMeta.ComputeSplineIndex(_tj, tjIU.first, tjIU.second);
            V_J_OFFSET = tjIU.first;
            /**
             * @attention: current R^3 trajectory is the velocity b-spline, whose
             * first zero derivative is the linear velocity, not the first order derivative!!!
             */
            Eigen::Vector3<T> LIN_VEL_J_BcToBc0InBc0;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                    sKnots + V_J_OFFSET, tjIU.second, _dtInv, &LIN_VEL_J_BcToBc0InBc0
            );
            Eigen::Vector3<T> pred = LIN_VEL_J_BcToBc0InBc0 - LIN_VEL_I_BcToBc0InBc0 - GRAVITY * (_tj - _ti);
            Eigen::Vector3<T> mes = -DEL_VEL_1.cast<T>() * POS_BiInBc + DEL_VEL_2.cast<T>();
            residuals.template block<3, 1>(0, 0) = T(_weight) * (pred - mes);

            return true;
        }
    };

    template<int Order>
    struct DiscretePreIntegrationFactor {
    private:
        ns_ctraj::SplineMeta<Order> _splineMeta;

        double _dt;

        Eigen::Matrix3d DEL_VEL_1;
        Eigen::Vector3d DEL_VEL_2;

        Eigen::Vector3d RadarVel_I;
        Eigen::Vector3d RadarVel_J;

        Eigen::Vector3d ANG_VEL_BcToBc0_I;
        Eigen::Vector3d ANG_VEL_BcToBc0_J;

        Sophus::SO3d SO3_BcToBc0_I;
        Sophus::SO3d SO3_BcToBc0_J;

        double _weight;

    public:
        explicit DiscretePreIntegrationFactor(double dt, Eigen::Matrix3d deltaVel1, Eigen::Vector3d deltaVel2,
                                              Eigen::Vector3d radarVelI, Eigen::Vector3d radarVelJ,
                                              Eigen::Vector3d BcVelI, Eigen::Vector3d BcVelJ,
                                              const Sophus::SO3d &BcToBc0_I, const Sophus::SO3d &BcToBc0_J,
                                              double weight)
                : _dt(dt), DEL_VEL_1(std::move(deltaVel1)), DEL_VEL_2(std::move(deltaVel2)),
                  RadarVel_I(std::move(radarVelI)), RadarVel_J(std::move(radarVelJ)),
                  ANG_VEL_BcToBc0_I(std::move(BcVelI)), ANG_VEL_BcToBc0_J(std::move(BcVelJ)),
                  SO3_BcToBc0_I(BcToBc0_I), SO3_BcToBc0_J(BcToBc0_J), _weight(weight) {}

        static auto Create(double dt, const Eigen::Matrix3d &deltaVel1, const Eigen::Vector3d &deltaVel2,
                           const Eigen::Vector3d &radarVelI, const Eigen::Vector3d &radarVelJ,
                           const Eigen::Vector3d &BcVelI, const Eigen::Vector3d &BcVelJ,
                           const Sophus::SO3d &BcToBc0_I, const Sophus::SO3d &BcToBc0_J, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<DiscretePreIntegrationFactor>(
                    new DiscretePreIntegrationFactor(
                            dt, deltaVel1, deltaVel2, radarVelI, radarVelJ,
                            BcVelI, BcVelJ, BcToBc0_I, BcToBc0_J, weight
                    )
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(DiscretePreIntegrationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ GRAVITY | POS_BiInBc | SO3_RjToBc | POS_RjInBc ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {
            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            std::size_t GRAVITY_OFFSET = 0;
            std::size_t POS_BiInBc_OFFSET = GRAVITY_OFFSET + 1;
            std::size_t SO3_RjToBc_OFFSET = POS_BiInBc_OFFSET + 1;
            std::size_t POS_RjInBc_OFFSET = SO3_RjToBc_OFFSET + 1;

            Eigen::Map<const Eigen::Vector3<T>> GRAVITY(sKnots[GRAVITY_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> POS_BiInBc(sKnots[POS_BiInBc_OFFSET]);
            Eigen::Map<Sophus::SO3<T> const> const SO3_RjToBc(sKnots[SO3_RjToBc_OFFSET]);
            Eigen::Map<const Eigen::Vector3<T>> POS_RjInBc(sKnots[POS_RjInBc_OFFSET]);

            Eigen::Vector3<T> LIN_VEL_I_BcToBc0InBc0 =
                    SO3_BcToBc0_I * SO3_RjToBc * RadarVel_I +
                    Sophus::SO3<T>::hat(SO3_BcToBc0_I * POS_RjInBc) * ANG_VEL_BcToBc0_I;

            Eigen::Vector3<T> LIN_VEL_J_BcToBc0InBc0 =
                    SO3_BcToBc0_J * SO3_RjToBc * RadarVel_J +
                    Sophus::SO3<T>::hat(SO3_BcToBc0_J * POS_RjInBc) * ANG_VEL_BcToBc0_J;

            Eigen::Vector3<T> pred = LIN_VEL_J_BcToBc0InBc0 - LIN_VEL_I_BcToBc0InBc0 - GRAVITY * _dt;
            Eigen::Vector3<T> mes = -DEL_VEL_1.cast<T>() * POS_BiInBc + DEL_VEL_2.cast<T>();
            residuals.template block<3, 1>(0, 0) = T(_weight) * (pred - mes);

            return true;
        }
    };
}

#endif //RIS_CALIB_PRE_INTEGRATION_FACTOR_HPP
