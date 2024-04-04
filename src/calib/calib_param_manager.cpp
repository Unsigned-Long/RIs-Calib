// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "filesystem"
#include "calib/calib_param_manager.h"
#include "spdlog/fmt/bundled/color.h"
#include "util/utils.hpp"

namespace ns_ris {

    CalibParamManager::CalibParamManager() : EXTRI(), TEMPORAL(), INTRI(), GRAVITY() {}

    CalibParamManager::Ptr CalibParamManager::Create() {
        return std::make_shared<CalibParamManager>();
    }

    void CalibParamManager::InitializeParametersFromConfigor() {
        spdlog::info("initialize calibration parameter manager using configor.");

        // extrinsic
        for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {
            // allocate memory
            EXTRI.SO3_BiToBc[topic] = Sophus::SO3d();
            EXTRI.POS_BiInBc[topic] = Eigen::Vector3d::Zero();
        }

        for (const auto &[topic, _]: Configor::DataStream::RadarTopics) {
            // allocate memory
            EXTRI.SO3_RjToBc[topic] = Sophus::SO3d();
            EXTRI.POS_RjInBc[topic] = Eigen::Vector3d::Zero();
        }
        // align to the 'z' axis
        GRAVITY = Eigen::Vector3d(0.0, 0.0, -Configor::Prior::GravityNorm);

        // temporal
        for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {
            // allocate memory
            TEMPORAL.TIME_OFFSET_BiToBc[topic] = 0.0;
        }

        for (const auto &[topic, _]: Configor::DataStream::RadarTopics) {
            TEMPORAL.TIME_OFFSET_RjToBc[topic] = 0.0;
        }

        // imu intrinsic

        for (const auto &[topic, _]: Configor::DataStream::IMUTopics) {
            // initialize intrinsic
            auto &IMU = INTRI.IMU[topic];
            IMU.Clear();
        }

    }

    void CalibParamManager::ShowParamStatus() {
        std::stringstream stream;
#define ITEM(name) fmt::format(fmt::emphasis::bold | fmt::fg(fmt::color::green), name)
#define PARAM(name) fmt::format(fmt::emphasis::bold, name)
#define STREAM_PACK(obj) stream << "-- " << obj << std::endl;

        constexpr
        std::size_t n = 74;

        STREAM_PACK(std::string(25, '-'))
        STREAM_PACK(ITEM("calibration parameters") << " --")
        STREAM_PACK(std::string(n, '-'))

        // -------------------------
        STREAM_PACK(ITEM("EXTRI"))
        // -------------------------
        STREAM_PACK("")

        for (const auto &[imuTopic, _]: CalibParamManager::EXTRI.SO3_BiToBc) {
            STREAM_PACK("IMU: '" << imuTopic << "'")
            const auto EULER_BiToBc = EXTRI.EULER_BiToBc_DEG(imuTopic);
            STREAM_PACK(PARAM("EULER_BiToBc: ") << FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_BiToBc(0), EULER_BiToBc(1), EULER_BiToBc(2)}))

            const auto POS_BiInBc = EXTRI.POS_BiInBc.at(imuTopic);
            STREAM_PACK(PARAM("  POS_BiInBc: ") << FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {POS_BiInBc(0), POS_BiInBc(1), POS_BiInBc(2)}))
            STREAM_PACK("")
        }

        for (const auto &[radarTopic, _]: CalibParamManager::EXTRI.SO3_RjToBc) {
            STREAM_PACK("Radar: '" << radarTopic << "'")
            const auto EULER_RjToBc = EXTRI.EULER_RjToBc_DEG(radarTopic);
            STREAM_PACK(PARAM("EULER_RjToBc: ") << FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_RjToBc(0), EULER_RjToBc(1), EULER_RjToBc(2)}))

            const auto POS_RjInBc = EXTRI.POS_RjInBc.at(radarTopic);
            STREAM_PACK(PARAM("  POS_RjInBc: ") << FormatValueVector<double>(
                    {"Px", "Py", "Pz"}, {POS_RjInBc(0), POS_RjInBc(1), POS_RjInBc(2)}))
            STREAM_PACK("")
        }
        STREAM_PACK(std::string(n, '-'))

        // ----------------------------
        STREAM_PACK(ITEM("TEMPORAL"))
        // ----------------------------
        STREAM_PACK("")
        for (const auto &[imuTopic, _]: CalibParamManager::EXTRI.SO3_BiToBc) {
            STREAM_PACK("IMU: '" << imuTopic << "'")
            const auto TIME_OFFSET_BiToBc = TEMPORAL.TIME_OFFSET_BiToBc.at(imuTopic);
            STREAM_PACK(fmt::format("{}: {:+011.6f} (s)", PARAM("TIME_OFFSET_BiToBc"), TIME_OFFSET_BiToBc))
            STREAM_PACK("")
        }
        for (const auto &[radarTopic, _]: CalibParamManager::EXTRI.SO3_RjToBc) {
            STREAM_PACK("Radar: '" << radarTopic << "'")
            const auto TIME_OFFSET_RjToBc = TEMPORAL.TIME_OFFSET_RjToBc.at(radarTopic);
            STREAM_PACK(fmt::format("{}: {:+011.6f} (s)", PARAM("TIME_OFFSET_RjToBc"), TIME_OFFSET_RjToBc))
            STREAM_PACK("")
        }
        STREAM_PACK(std::string(n, '-'))
        // -------------------------
        STREAM_PACK(ITEM("INTRI"))
        // -------------------------
        STREAM_PACK("")
        for (const auto &[imuTopic, _]: CalibParamManager::EXTRI.SO3_BiToBc) {
            STREAM_PACK("IMU: '" << imuTopic << "'")
            const auto &ACCE = INTRI.IMU.at(imuTopic).ACCE;
            const auto &GYRO = INTRI.IMU.at(imuTopic).GYRO;
            // imu
            STREAM_PACK(PARAM("ACCE      BIAS: ") << FormatValueVector<double>(
                    {"Bx", "By", "Bz"}, {ACCE.BIAS(0), ACCE.BIAS(1), ACCE.BIAS(2)}))
            STREAM_PACK(PARAM("ACCE MAP COEFF: ") << FormatValueVector<double>(
                    {"00", "11", "22"},
                    {ACCE.MAP_COEFF(0), ACCE.MAP_COEFF(1), ACCE.MAP_COEFF(2)}))
            STREAM_PACK(PARAM("                ") << FormatValueVector<double>(
                    {"01", "02", "12"},
                    {ACCE.MAP_COEFF(3), ACCE.MAP_COEFF(4), ACCE.MAP_COEFF(5)}))

            STREAM_PACK("")

            STREAM_PACK(PARAM("GYRO      BIAS: ") << FormatValueVector<double>(
                    {"Bx", "By", "Bz"}, {GYRO.BIAS(0), GYRO.BIAS(1), GYRO.BIAS(2)}))
            STREAM_PACK(PARAM("GYRO MAP COEFF: ") << FormatValueVector<double>(
                    {"00", "11", "22"},
                    {GYRO.MAP_COEFF(0), GYRO.MAP_COEFF(1), GYRO.MAP_COEFF(2)}))
            STREAM_PACK(PARAM("                ") << FormatValueVector<double>(
                    {"01", "02", "12"},
                    {GYRO.MAP_COEFF(3), GYRO.MAP_COEFF(4), GYRO.MAP_COEFF(5)}))

            STREAM_PACK("")

            const auto EULER_AtoG = INTRI.IMU.at(imuTopic).EULER_AtoG_DEG();
            STREAM_PACK(PARAM("EULER AtoG DEG: ") << FormatValueVector<double>(
                    {"Xr", "Yp", "Zy"}, {EULER_AtoG(0), EULER_AtoG(1), EULER_AtoG(2)}))
            STREAM_PACK("")
        }
        STREAM_PACK(std::string(n, '-'))

        // ------------------------------
        STREAM_PACK(ITEM("OTHER FIELDS"))
        // ------------------------------
        STREAM_PACK("")
        STREAM_PACK(PARAM("GRAVITY IN MAP: ") << FormatValueVector<double>(
                {"Gx", "Gy", "Gz"}, {GRAVITY(0), GRAVITY(1), GRAVITY(2)}))
        STREAM_PACK(std::string(n, '-'))

        spdlog::info("the detail calibration parameters are below: \n{}", stream.str());

#undef ITEM
#undef PARAM
    }

    void CalibParamManager::VisualizationSensors(ns_viewer::Viewer &viewer) const {
        auto SE3_BcToBc = Sophus::SE3f();
        auto centerIMU = ns_viewer::IMU::Create(
                ns_viewer::Posef(SE3_BcToBc.so3().matrix(), SE3_BcToBc.translation()), 0.1,
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 1.0f)
        );
        viewer.AddEntity(centerIMU);

        for (const auto &[imuTopic, _]: CalibParamManager::EXTRI.SO3_BiToBc) {
            auto BiToBc = EXTRI.SE3_BiToBc(imuTopic).cast<float>();
            auto imu = ns_viewer::IMU::Create(
                    ns_viewer::Posef(BiToBc.so3().matrix(), BiToBc.translation()), 0.1
            );
            auto line = ns_viewer::Line::Create(
                    Eigen::Vector3f::Zero(), BiToBc.translation().cast<float>(), ns_viewer::Colour::Black()
            );
            viewer.AddEntity({imu, line});
        }

        for (const auto &[radarTopic, _]: CalibParamManager::EXTRI.SO3_RjToBc) {
            auto RjToBc = EXTRI.SE3_RjToBc(radarTopic).cast<float>();
            auto radar = ns_viewer::Radar::Create(
                    ns_viewer::Posef(RjToBc.so3().matrix(), RjToBc.translation()), 0.1, ns_viewer::Colour::Blue()
            );
            auto line = ns_viewer::Line::Create(
                    Eigen::Vector3f::Zero(), RjToBc.translation().cast<float>(), ns_viewer::Colour::Black()
            );
            viewer.AddEntity({radar, line});
        }
    }

    CalibParamManager::Ptr
    CalibParamManager::AlignParamToNewSensor(const Sophus::SE3d &SE3_BcToNew, double TF_BcToNew) const {
        auto alignedParam = CalibParamManager::Create();
        *alignedParam = *this;
        for (const auto &[imuTopic, _]: alignedParam->EXTRI.SO3_BiToBc) {
            auto alignedSE3 = SE3_BcToNew * alignedParam->EXTRI.SE3_BiToBc(imuTopic);

            alignedParam->EXTRI.SO3_BiToBc.at(imuTopic) = alignedSE3.so3();
            alignedParam->EXTRI.POS_BiInBc.at(imuTopic) = alignedSE3.translation();

            alignedParam->TEMPORAL.TIME_OFFSET_BiToBc.at(imuTopic) += TF_BcToNew;
        }
        for (const auto &[radarTopic, _]: alignedParam->EXTRI.SO3_RjToBc) {
            auto alignedSE3 = SE3_BcToNew * alignedParam->EXTRI.SE3_RjToBc(radarTopic);

            alignedParam->EXTRI.SO3_RjToBc.at(radarTopic) = alignedSE3.so3();
            alignedParam->EXTRI.POS_RjInBc.at(radarTopic) = alignedSE3.translation();

            alignedParam->TEMPORAL.TIME_OFFSET_RjToBc.at(radarTopic) += TF_BcToNew;
        }
        return alignedParam;
    }

    CalibParamManager::Ptr CalibParamManager::AlignParamToNewSensor(const std::string &topic) const {
        return AlignParamToNewSensor(
                this->EXTRI.SE3_BiToBc(topic).inverse(), -this->TEMPORAL.TIME_OFFSET_BiToBc.at(topic)
        );
    }

    void CalibParamManager::Save(const std::string &filename, CerealArchiveType::Enum archiveType) const {
        std::ofstream file(filename, std::ios::out);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *this));
    }

    CalibParamManager::Ptr CalibParamManager::Load(const std::string &filename, CerealArchiveType::Enum archiveType) {
        auto calibParamManager = CalibParamManager::Create();
        std::ifstream file(filename, std::ios::in);
        auto ar = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *calibParamManager));
        return calibParamManager;
    }
}
