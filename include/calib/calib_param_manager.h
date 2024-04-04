// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_CALIB_PARAM_MANAGER_H
#define RIS_CALIB_CALIB_PARAM_MANAGER_H

#include "sophus/se3.hpp"
#include "ctraj/view/traj_viewer.h"
#include "cereal/types/map.hpp"
#include "spdlog/spdlog.h"
#include "config/configor.h"

namespace ns_ris {
    struct CalibParamManager {
    public:
        using Ptr = std::shared_ptr<CalibParamManager>;

    public:
        // trans radian angle to degree angle
        constexpr static double RAD_TO_DEG = 180.0 / M_PI;
        // trans degree angle to radian angle
        constexpr static double DEG_TO_RAD = M_PI / 180.0;

        // extrinsic
        struct {
            std::map<std::string, Sophus::SO3d> SO3_BiToBc;
            std::map<std::string, Eigen::Vector3d> POS_BiInBc;

            std::map<std::string, Sophus::SO3d> SO3_RjToBc;
            std::map<std::string, Eigen::Vector3d> POS_RjInBc;

            // lie algebra vector space se3
            [[nodiscard]] Sophus::SE3d SE3_RjToBc(const std::string &radarTopic) const {
                return {SO3_RjToBc.at(radarTopic), POS_RjInBc.at(radarTopic)};
            }

            [[nodiscard]] Sophus::SE3d SE3_BiToBc(const std::string &imuTopic) const {
                return {SO3_BiToBc.at(imuTopic), POS_BiInBc.at(imuTopic)};
            }

            // quaternion
            [[nodiscard]] Eigen::Quaterniond Q_RjToBc(const std::string &radarTopic) const {
                return SO3_RjToBc.at(radarTopic).unit_quaternion();
            }

            [[nodiscard]] Eigen::Quaterniond Q_BiToBc(const std::string &imuTopic) const {
                return SO3_BiToBc.at(imuTopic).unit_quaternion();
            }

            // the euler angles [radian and degree format]
            [[nodiscard]] Eigen::Vector3d EULER_RjToBc_RAD(const std::string &radarTopic) const {
                return Q_RjToBc(radarTopic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_RjToBc_DEG(const std::string &radarTopic) const {
                auto euler = EULER_RjToBc_RAD(radarTopic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            [[nodiscard]] Eigen::Vector3d EULER_BiToBc_RAD(const std::string &imuTopic) const {
                return Q_BiToBc(imuTopic).toRotationMatrix().eulerAngles(0, 1, 2);
            }

            [[nodiscard]] Eigen::Vector3d EULER_BiToBc_DEG(const std::string &imuTopic) const {
                auto euler = EULER_BiToBc_RAD(imuTopic);
                for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                return euler;
            }

            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;

#define SAVE_EXTRI_INFO(param) infoMap.insert(std::make_pair(param.data(), #param));

                // imu
                for (const auto &[topic, data]: SO3_BiToBc) {
                    infoMap.insert({data.data(), "SO3_BiToBc[" + topic + "]"});
                }
                for (const auto &[topic, data]: POS_BiInBc) {
                    infoMap.insert({data.data(), "POS_BiInBc[" + topic + "]"});
                }

                // radar
                for (const auto &[topic, data]: SO3_RjToBc) {
                    infoMap.insert({data.data(), "SO3_RjToBc[" + topic + "]"});
                }
                for (const auto &[topic, data]: POS_RjInBc) {
                    infoMap.insert({data.data(), "POS_RjInBc[" + topic + "]"});
                }

#undef SAVE_EXTRI_INFO

                return infoMap;
            }


            std::vector<Sophus::SO3d *> SO3_BiToBc_AddressVec() {
                std::vector<Sophus::SO3d *> addressVec(SO3_BiToBc.size());
                std::transform(SO3_BiToBc.begin(), SO3_BiToBc.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

            std::vector<Eigen::Vector3d *> POS_BiInBc_AddressVec() {
                std::vector<Eigen::Vector3d *> addressVec(POS_BiInBc.size());
                std::transform(POS_BiInBc.begin(), POS_BiInBc.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }

        public:
            // Serialization
            template<class Archive>
            void serialize(Archive &archive) {
                archive(CEREAL_NVP(SO3_BiToBc), CEREAL_NVP(POS_BiInBc),
                        CEREAL_NVP(SO3_RjToBc), CEREAL_NVP(POS_RjInBc));
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } EXTRI;

        // temporal
        struct {
            std::map<std::string, double> TIME_OFFSET_BiToBc;
            std::map<std::string, double> TIME_OFFSET_RjToBc;

            std::vector<double *> TIME_OFFSET_BiToBc_AddressVec() {
                std::vector<double *> addressVec(TIME_OFFSET_BiToBc.size());
                std::transform(TIME_OFFSET_BiToBc.begin(), TIME_OFFSET_BiToBc.end(), addressVec.begin(), [](auto &p) {
                    return &(p.second);
                });
                return addressVec;
            }


            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;

#define SAVE_TEMPORAL_INFO(param) infoMap.insert(std::make_pair(&param, #param));

                for (const auto &[topic, data]: TIME_OFFSET_BiToBc) {
                    infoMap.insert({&data, "TIME_OFFSET_BiToBc[" + topic + "]"});
                }

                for (const auto &[topic, data]: TIME_OFFSET_RjToBc) {
                    infoMap.insert({&data, "TIME_OFFSET_RjToBc[" + topic + "]"});
                }

#undef SAVE_TEMPORAL_INFO

                return infoMap;
            }

        public:
            // Serialization
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(TIME_OFFSET_BiToBc), CEREAL_NVP(TIME_OFFSET_RjToBc));
            }
        } TEMPORAL;

        // intrinsic
        struct {
            struct IMUParamPack {
                struct BiasMapCoeff {
                    Eigen::Vector3d BIAS;
                    /**
                     * MAP_COEFF: [v1, v2, v3, v4, v5, v6]^T
                     * mapMatrix:
                     *   v1 & v4 & v5
                     *    0 & v2 & v6
                     *    0 &  0 & v3
                     * f(measure) = mapMat * f(real) + bias
                     */
                    Eigen::Vector6d MAP_COEFF;

                    // organize the vector to a matrix
                    [[nodiscard]] Eigen::Matrix3d MapMatrix() const {
                        Eigen::Matrix3d mat;
                        mat.diagonal() = Eigen::Map<const Eigen::Vector3d>(MAP_COEFF.data(), 3);
                        mat(0, 1) = *(MAP_COEFF.data() + 3);
                        mat(0, 2) = *(MAP_COEFF.data() + 4);
                        mat(1, 2) = *(MAP_COEFF.data() + 5);
                        return Eigen::Matrix3d(MAP_COEFF.data());
                    }

                    void Clear() {
                        BIAS = Eigen::Vector3d::Zero();
                        MAP_COEFF = Eigen::Vector6d::Zero();
                        MAP_COEFF(0) = 1.0;
                        MAP_COEFF(1) = 1.0;
                        MAP_COEFF(2) = 1.0;
                    }

                    // Serialization
                    template<class Archive>
                    void serialize(Archive &archive) {
                        archive(CEREAL_NVP(BIAS), CEREAL_NVP(MAP_COEFF));
                    }

                public:
                    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                };

                BiasMapCoeff ACCE, GYRO;
                Sophus::SO3d SO3_AtoG;

                void Clear() {
                    ACCE.Clear();
                    GYRO.Clear();
                    SO3_AtoG = Sophus::SO3d();
                }

                // quaternion
                [[nodiscard]] Eigen::Quaterniond Q_AtoG() const {
                    return SO3_AtoG.unit_quaternion();
                }

                // euler angles
                [[nodiscard]] Eigen::Vector3d EULER_AtoG_RAD() const {
                    return Q_AtoG().toRotationMatrix().eulerAngles(0, 1, 2);
                }

                [[nodiscard]] Eigen::Vector3d EULER_AtoG_DEG() const {
                    auto euler = EULER_AtoG_RAD();
                    for (int i = 0; i != 3; ++i) { euler(i) *= CalibParamManager::RAD_TO_DEG; }
                    return euler;
                }

                // Serialization
                template<class Archive>
                void serialize(Archive &archive) {
                    archive(CEREAL_NVP(ACCE), CEREAL_NVP(GYRO), CEREAL_NVP(SO3_AtoG));
                }
            };

            // [ topic, param pack ]
            std::map<std::string, IMUParamPack> IMU;


            std::map<const double *, std::string> GetParamAddressWithDesc() {
                std::map<const double *, std::string> infoMap;
                // IMU
                for (const auto &[topic, pack]: IMU) {
                    infoMap.insert({pack.ACCE.MAP_COEFF.data(), "IMU.ACCE.MAP_COEFF[" + topic + "]"});
                    infoMap.insert({pack.ACCE.BIAS.data(), "IMU.ACCE.BIAS[" + topic + "]"});
                    infoMap.insert({pack.GYRO.MAP_COEFF.data(), "IMU.GYRO.MAP_COEFF[" + topic + "]"});
                    infoMap.insert({pack.GYRO.BIAS.data(), "IMU.GYRO.BIAS[" + topic + "]"});
                    infoMap.insert({pack.SO3_AtoG.data(), "IMU.SO3_AtoG[" + topic + "]"});
                }

                return infoMap;
            }

        public:
            // Serialization
            template<class Archive>
            void serialize(Archive &archive) {
                archive(CEREAL_NVP(IMU));
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        } INTRI;

        // S2Manifold
        Eigen::Vector3d GRAVITY;

    public:

        // the constructor
        explicit CalibParamManager();

        // the creator
        static CalibParamManager::Ptr Create();

        // save the parameters to file using cereal library
        template<class CerealArchiveType=CerealArchiveType::YAML>
        void Save(const std::string &filename) const {
            std::ofstream file(filename, std::ios::out);
            auto ar = GetOutputArchive<CerealArchiveType>(file);

            (*ar)(cereal::make_nvp("CalibParam", *this));
        }

        // load the parameters from file using cereal library
        template<class CerealArchiveType=CerealArchiveType::YAML>
        static CalibParamManager::Ptr Load(const std::string &filename) {
            auto calibParamManager = CalibParamManager::Create();
            std::ifstream file(filename, std::ios::in);
            auto ar = GetInputArchive<CerealArchiveType>(file);

            (*ar)(cereal::make_nvp("CalibParam", *calibParamManager));
            return calibParamManager;
        }

        // save the parameters to file using cereal library
        void Save(const std::string &filename, CerealArchiveType::Enum archiveType) const;

        // load the parameters from file using cereal library
        static CalibParamManager::Ptr Load(const std::string &filename, CerealArchiveType::Enum archiveType);

        // print the parameters in the console
        void ShowParamStatus();

        void VisualizationSensors(ns_viewer::Viewer &viewer) const;

        // set the params to the init values, the intrinsic coeff of camera will load from the config file
        // make sure load and check config before initialize the parameters
        void InitializeParametersFromConfigor();

        [[nodiscard]] CalibParamManager::Ptr
        AlignParamToNewSensor(const Sophus::SE3d &SE3_BcToNew, double TF_BcToNew) const;

        [[nodiscard]] CalibParamManager::Ptr AlignParamToNewSensor(const std::string &topic) const;

    public:
        // Serialization
        template<class Archive>
        void serialize(Archive &archive) {
            archive(CEREAL_NVP(EXTRI), CEREAL_NVP(TEMPORAL), CEREAL_NVP(INTRI), CEREAL_NVP(GRAVITY));
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif //RIS_CALIB_CALIB_PARAM_MANAGER_H
