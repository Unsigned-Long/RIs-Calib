// Copyright (c) 2023. Created on 7/25/23 1:33 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_ERROR_STATISTICS_HPP
#define RIS_CALIB_ERROR_STATISTICS_HPP

#include "calib/calib_param_manager.h"
#include "util/utils.hpp"

namespace ns_ris {
    struct ErrorStatistic {
        static CalibParamManager::Ptr
        Perform(const CalibParamManager::Ptr &srcParam, const CalibParamManager::Ptr &tarParam) {
            auto resParam = CalibParamManager::Create();

            // extrinsic
            for (const auto &[topic, SO3_BiToBc]: srcParam->EXTRI.SO3_BiToBc) {
                auto error = SO3_BiToBc.inverse() * tarParam->EXTRI.SO3_BiToBc.at(topic);
                resParam->EXTRI.SO3_BiToBc[topic] = error;
            }
            for (const auto &[topic, SO3_RjToBc]: srcParam->EXTRI.SO3_RjToBc) {
                auto error = SO3_RjToBc.inverse() * tarParam->EXTRI.SO3_RjToBc.at(topic);
                resParam->EXTRI.SO3_RjToBc[topic] = error;
            }
            for (const auto &[topic, POS_BiInBc]: srcParam->EXTRI.POS_BiInBc) {
                Eigen::Vector3d error = POS_BiInBc - tarParam->EXTRI.POS_BiInBc.at(topic);
                resParam->EXTRI.POS_BiInBc[topic] = error;
            }
            for (const auto &[topic, POS_RjInBc]: srcParam->EXTRI.POS_RjInBc) {
                Eigen::Vector3d error = POS_RjInBc - tarParam->EXTRI.POS_RjInBc.at(topic);
                resParam->EXTRI.POS_RjInBc[topic] = error;
            }
            // temporal
            for (const auto &[topic, TIME_OFFSET_BiToBc]: srcParam->TEMPORAL.TIME_OFFSET_BiToBc) {
                auto error = TIME_OFFSET_BiToBc - tarParam->TEMPORAL.TIME_OFFSET_BiToBc.at(topic);
                resParam->TEMPORAL.TIME_OFFSET_BiToBc[topic] = error;
            }
            for (const auto &[topic, TIME_OFFSET_RjToBc]: srcParam->TEMPORAL.TIME_OFFSET_RjToBc) {
                auto error = TIME_OFFSET_RjToBc - tarParam->TEMPORAL.TIME_OFFSET_RjToBc.at(topic);
                resParam->TEMPORAL.TIME_OFFSET_RjToBc[topic] = error;
            }
            // intrinsic
            for (const auto &[topic, IMU]: srcParam->INTRI.IMU) {
                // bias
                Eigen::Vector3d acceBias = IMU.ACCE.BIAS - tarParam->INTRI.IMU.at(topic).ACCE.BIAS;
                resParam->INTRI.IMU[topic].ACCE.BIAS = acceBias;
                Eigen::Vector3d gyroBias = IMU.GYRO.BIAS - tarParam->INTRI.IMU.at(topic).GYRO.BIAS;
                resParam->INTRI.IMU[topic].GYRO.BIAS = gyroBias;

                // SO3_AtoG
                auto SO3_AtoG_ERROR = IMU.SO3_AtoG.inverse() * tarParam->INTRI.IMU.at(topic).SO3_AtoG;
                resParam->INTRI.IMU[topic].SO3_AtoG = SO3_AtoG_ERROR;

                // mapping matrix
                Eigen::Vector6d acceMapError = IMU.ACCE.MAP_COEFF - tarParam->INTRI.IMU.at(topic).ACCE.MAP_COEFF;
                resParam->INTRI.IMU[topic].ACCE.MAP_COEFF = acceMapError;
                Eigen::Vector6d gyroMapError = IMU.GYRO.MAP_COEFF - tarParam->INTRI.IMU.at(topic).GYRO.MAP_COEFF;
                resParam->INTRI.IMU[topic].GYRO.MAP_COEFF = gyroMapError;
            }

            return resParam;
        }

        static std::vector<CalibParamManager::Ptr>
        PerformForParamsIter(const std::string &paramsIterDir, const std::string &gtParamPath) {
            // ------------------------------
            // get dst files in the directory
            // ------------------------------
            auto files = FilesInDir(paramsIterDir);
            files.erase(std::remove_if(files.begin(), files.end(), [](const std::string &str) {
                auto strVec = SplitString(SplitString(str, '/').back(), '.');
                return strVec.back() != "json" || strVec.front().substr(0, 5) != "param";
            }), files.end());

            // sort the tarParamFile vector
            std::sort(files.begin(), files.end(), [](const std::string &str1, const std::string &str2) {
                auto file1 = SplitString(SplitString(str1, '/').back(), '.').front();
                auto file2 = SplitString(SplitString(str2, '/').back(), '.').front();
                auto idx1 = std::stoi(SplitString(file1, '_').back());
                auto idx2 = std::stoi(SplitString(file2, '_').back());
                return idx1 < idx2;
            });
            std::vector<CalibParamManager::Ptr> params;
            for (const auto &filename: files) {
                spdlog::info("process param file named '{}'...", filename);
                auto tarParam = CalibParamManager::Load<CerealArchiveType::YAML>(gtParamPath);
                // aligned st parameters to the first imu
                auto srcParam = CalibParamManager::Load<CerealArchiveType::JSON>(filename)->AlignParamToNewSensor(
                        tarParam->EXTRI.SO3_BiToBc.cbegin()->first
                );
                params.push_back(Perform(srcParam, tarParam));
            }
            return params;
        }

        static void
        SaveSTParamsAsRawJson(const std::vector<CalibParamManager::Ptr> &params, const std::string &saveFilename) {
            std::map<std::string, std::vector<Eigen::Vector3d>> imuEulerError, radarEulerError, imuBaError, imuBgError, imuPosError, radarPosError;
            std::map<std::string, std::vector<double>> imuTempError, radarTempError;
            for (const auto &param: params) {
                for (const auto &[topic, _]: param->EXTRI.SO3_BiToBc) {
                    imuEulerError[topic].push_back(param->EXTRI.EULER_BiToBc_DEG(topic));
                    imuPosError[topic].push_back(param->EXTRI.POS_BiInBc.at(topic));

                    imuTempError[topic].push_back(param->TEMPORAL.TIME_OFFSET_BiToBc.at(topic));

                    imuBaError[topic].push_back(param->INTRI.IMU.at(topic).ACCE.BIAS);
                    imuBgError[topic].push_back(param->INTRI.IMU.at(topic).GYRO.BIAS);
                }
                for (const auto &[topic, _]: param->EXTRI.SO3_RjToBc) {
                    radarEulerError[topic].push_back(param->EXTRI.EULER_RjToBc_DEG(topic));
                    radarPosError[topic].push_back(param->EXTRI.POS_RjInBc.at(topic));

                    radarTempError[topic].push_back(param->TEMPORAL.TIME_OFFSET_RjToBc.at(topic));
                }
            }
            std::ofstream of(saveFilename);
            cereal::JSONOutputArchive ar(of);
            ar(CEREAL_NVP(imuEulerError), CEREAL_NVP(imuPosError), CEREAL_NVP(imuTempError),
               CEREAL_NVP(imuBaError), CEREAL_NVP(imuBgError),
               CEREAL_NVP(radarEulerError), CEREAL_NVP(radarPosError), CEREAL_NVP(radarTempError));
        }
    };
}

#endif //RIS_CALIB_ERROR_STATISTICS_HPP
