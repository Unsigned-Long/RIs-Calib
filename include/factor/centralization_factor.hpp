// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_CENTRALIZATION_FACTOR_HPP
#define RIS_CALIB_CENTRALIZATION_FACTOR_HPP

#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_ris {
    struct SO3CentralizationFactor {
    private:
        std::size_t _blockSize;
        double _weight;

    public:
        explicit SO3CentralizationFactor(std::size_t blockSize, double weight)
                : _blockSize(blockSize), _weight(weight) {}

        static auto Create(std::size_t blockSize, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<SO3CentralizationFactor>(
                    new SO3CentralizationFactor(blockSize, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(SO3CentralizationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ SO3_BiToBc | ... | SO3_BiToBc | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            Eigen::Vector3<T> normSO3 = Eigen::Vector3<T>::Zero();
            for (int SO3_BiToBc_OFFSET = 0; SO3_BiToBc_OFFSET < static_cast<int>(_blockSize); ++SO3_BiToBc_OFFSET) {
                Eigen::Map<Sophus::SO3<T> const> const SO3_BiToBc(sKnots[SO3_BiToBc_OFFSET]);
                normSO3 += SO3_BiToBc.log();
            }

            residuals.template block<3, 1>(0, 0) = T(_weight) * normSO3;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct POSCentralizationFactor {
    private:
        std::size_t _blockSize;
        double _weight;

    public:
        explicit POSCentralizationFactor(std::size_t blockSize, double weight)
                : _blockSize(blockSize), _weight(weight) {}

        static auto Create(std::size_t blockSize, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<POSCentralizationFactor>(
                    new POSCentralizationFactor(blockSize, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(POSCentralizationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ POS_BiInBc | ... | POS_BiInBc | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

            Eigen::Vector3<T> normPOS = Eigen::Vector3<T>::Zero();
            for (int POS_BiInBc_OFFSET = 0; POS_BiInBc_OFFSET < static_cast<int>(_blockSize); ++POS_BiInBc_OFFSET) {
                Eigen::Map<Eigen::Vector3<T> const> const POS_BiInBc(sKnots[POS_BiInBc_OFFSET]);
                normPOS += POS_BiInBc;
            }

            residuals.template block<3, 1>(0, 0) = T(_weight) * normPOS;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct TimeOffsetCentralizationFactor {
    private:
        std::size_t _blockSize;
        double _weight;

    public:
        explicit TimeOffsetCentralizationFactor(std::size_t blockSize, double weight)
                : _blockSize(blockSize), _weight(weight) {}

        static auto Create(std::size_t blockSize, double weight) {
            return new ceres::DynamicAutoDiffCostFunction<TimeOffsetCentralizationFactor>(
                    new TimeOffsetCentralizationFactor(blockSize, weight)
            );
        }

        static std::size_t TypeHashCode() {
            return typeid(TimeOffsetCentralizationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ TIME_OFFSET_BiToBc | ... | TIME_OFFSET_BiToBc | ... ]
         */
        template<class T>
        bool operator()(T const *const *sKnots, T *sResiduals) const {

            Eigen::Map<Eigen::Vector1<T>>
                    residuals(sResiduals);

            Eigen::Vector1<T> normTimeOffset = Eigen::Vector1<T>::Zero();
            for (int TIME_OFFSET_BiToBc_OFFSET = 0;
                 TIME_OFFSET_BiToBc_OFFSET < static_cast<int>(_blockSize); ++TIME_OFFSET_BiToBc_OFFSET) {
                Eigen::Map<Eigen::Vector1<T> const> const TIME_OFFSET_BiToBc(sKnots[TIME_OFFSET_BiToBc_OFFSET]);
                normTimeOffset += TIME_OFFSET_BiToBc;
            }

            residuals.template block<1, 1>(0, 0) = T(_weight) * normTimeOffset;

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //RIS_CALIB_CENTRALIZATION_FACTOR_HPP
