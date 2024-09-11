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

#ifndef RIS_CALIB_SPLINE_ESTIMATOR_H
#define RIS_CALIB_SPLINE_ESTIMATOR_H

#include "ctraj/core/trajectory_estimator.h"
#include "config/configor.h"
#include "sensor/imu.h"
#include "config/configor.h"
#include "calib/calib_param_manager.h"
#include "sensor/radar.h"
#include "lm_equation.h"

namespace ns_ris {
    using ns_ctraj::Posed;

    struct OptOption {
        // myenumGenor Option OPT_SO3 OPT_VEL OPT_SO3_BiToBc OPT_POS_BiInBc OPT_SO3_RjToBc OPT_POS_RjInBc OPT_TIME_OFFSET_BiToBc OPT_TIME_OFFSET_RjToBc OPT_GYRO_BIAS OPT_GYRO_MAP_COEFF OPT_ACCE_BIAS OPT_ACCE_MAP_COEFF OPT_SO3_AtoG OPT_GRAVITY
        enum Option : std::uint32_t {
            /**
             * @brief options
             */
            NONE = 1 << 0,

            OPT_SO3 = 1 << 1,
            OPT_VEL = 1 << 2,

            OPT_SO3_BiToBc = 1 << 3,
            OPT_POS_BiInBc = 1 << 4,

            OPT_SO3_RjToBc = 1 << 5,
            OPT_POS_RjInBc = 1 << 6,

            OPT_TIME_OFFSET_BiToBc = 1 << 7,
            OPT_TIME_OFFSET_RjToBc = 1 << 8,

            OPT_GYRO_BIAS = 1 << 9,
            OPT_GYRO_MAP_COEFF = 1 << 10,

            OPT_ACCE_BIAS = 1 << 11,
            OPT_ACCE_MAP_COEFF = 1 << 12,

            OPT_SO3_AtoG = 1 << 13,

            OPT_GRAVITY = 1 << 14,

            ALL = OPT_SO3 | OPT_VEL | OPT_SO3_BiToBc | OPT_POS_BiInBc | OPT_SO3_RjToBc | OPT_POS_RjInBc |
                  OPT_TIME_OFFSET_BiToBc | OPT_TIME_OFFSET_RjToBc | OPT_GYRO_BIAS | OPT_GYRO_MAP_COEFF |
                  OPT_ACCE_BIAS | OPT_ACCE_MAP_COEFF | OPT_SO3_AtoG | OPT_GRAVITY
        };

        static bool IsOptionWith(std::uint32_t desired, std::uint32_t curOption) {
            return (desired == (desired & curOption));
        }

        /**
         * @brief override operator '<<' for type 'Option'
         */
        friend std::ostream &operator<<(std::ostream &os, const Option &curOption) {
            std::stringstream stream;
            int count = 0;
            if (IsOptionWith(OPT_SO3, curOption)) {
                stream << "OPT_SO3";
                ++count;
            }
            if (IsOptionWith(OPT_VEL, curOption)) {
                stream << " | OPT_VEL";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_BiToBc, curOption)) {
                stream << " | OPT_SO3_BiToBc";
                ++count;
            }
            if (IsOptionWith(OPT_POS_BiInBc, curOption)) {
                stream << " | OPT_POS_BiInBc";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_RjToBc, curOption)) {
                stream << " | OPT_SO3_RjToBc";
                ++count;
            }
            if (IsOptionWith(OPT_POS_RjInBc, curOption)) {
                stream << " | OPT_POS_RjInBc";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_OFFSET_BiToBc, curOption)) {
                stream << " | OPT_TIME_OFFSET_BiToBc";
                ++count;
            }
            if (IsOptionWith(OPT_TIME_OFFSET_RjToBc, curOption)) {
                stream << " | OPT_TIME_OFFSET_RjToBc";
                ++count;
            }
            if (IsOptionWith(OPT_GYRO_BIAS, curOption)) {
                stream << " | OPT_GYRO_BIAS";
                ++count;
            }
            if (IsOptionWith(OPT_GYRO_MAP_COEFF, curOption)) {
                stream << " | OPT_GYRO_MAP_COEFF";
                ++count;
            }
            if (IsOptionWith(OPT_ACCE_BIAS, curOption)) {
                stream << " | OPT_ACCE_BIAS";
                ++count;
            }
            if (IsOptionWith(OPT_ACCE_MAP_COEFF, curOption)) {
                stream << " | OPT_ACCE_MAP_COEFF";
                ++count;
            }
            if (IsOptionWith(OPT_SO3_AtoG, curOption)) {
                stream << " | OPT_SO3_AtoG";
                ++count;
            }
            if (IsOptionWith(OPT_GRAVITY, curOption)) {
                stream << " | OPT_GRAVITY";
                ++count;
            }
            if (count == 0) {
                os << "NONE";
            } else if (count == 14) {
                os << "ALL";
            } else {
                std::string str = stream.str();
                if (str.at(1) == '|') {
                    str = str.substr(3, str.size() - 3);
                }
                os << str;
            }
            return os;
        };
    };

    class SplineEstimator : public ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder> {
    public:
        using Ptr = std::shared_ptr<SplineEstimator>;
        using Parent = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>;
        using SplineMeta = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
        using Trajectory = ns_ctraj::Trajectory<Configor::Prior::SplineOrder>;

    protected:
        CalibParamManager::Ptr _calibParam;

        std::vector<CeresFactor> _factors;
    public:
        explicit SplineEstimator(const Trajectory::Ptr &trajectory, CalibParamManager::Ptr calibParam);

        static Ptr Create(const Trajectory::Ptr &trajectory, const CalibParamManager::Ptr &calibParam);

        LMEquation Evaluate(const std::map<const double *, std::string> &targetParamsInfoMap);

        LMEquation Evaluate(const std::initializer_list<std::map<const double *, std::string>> &targetParamsInfoMaps);

    public:
        void AddIMUAcceMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                   int option, double acceWeight);

        void AddIMUGyroMeasurement(const IMUFrame::Ptr &imuFrame, const std::string &topic,
                                   int option, double gyroWeight);

        void AddRadarMeasurement(const RadarTarget::Ptr &radarFrame, const std::string &topic,
                                 int option, double weight);

        void AddVelPreIntegration(const std::vector<IMUFrame::Ptr> &imuMes, const std::string &imuTopic,
                                  double ti, double tj, int option, double weight);

        void AddDiscreteVelPreIntegration(const std::vector<IMUFrame::Ptr> &imuMes, const std::string &imuTopic,
                                          const std::string &radarTopic, double ti, double tj,
                                          const Eigen::Vector3d &radarVelI, const Eigen::Vector3d &radarVelJ,
                                          int option, double weight);

        void FixFirSO3ControlPoint();

        void AddSO3Centralization(const std::vector<Sophus::SO3d *> &SO3_StoRef,
                                  double weight, std::uint32_t option);

        void AddPOSCentralization(const std::vector<Eigen::Vector3d *> &POS_SinRef,
                                  double weight, std::uint32_t option);

        void AddTimeOffsetCentralization(const std::vector<double *> &TIME_OFFSET_StoRef,
                                         double weight, std::uint32_t option);

    protected:
        static auto ExtractRange(const std::vector<IMUFrame::Ptr> &data, double st, double et) {
            auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() > st;
            });
            auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                return frame->GetTimestamp() < et;
            }).base();
            return std::pair(sIter, eIter);
        }

        template<typename CostFunctor, int Stride = 4>
        ceres::ResidualBlockId
        AddResidualBlockToProblem(ceres::DynamicAutoDiffCostFunction<CostFunctor, Stride> *costFunc,
                                  ceres::LossFunction *lossFunc, const std::vector<double *> &paramBlocks) {
            _factors.emplace_back(costFunc, paramBlocks, typeid(CostFunctor).hash_code());
            return Problem::AddResidualBlock(costFunc, lossFunc, paramBlocks);
        }
    };
}


#endif //RIS_CALIB_SPLINE_ESTIMATOR_H
