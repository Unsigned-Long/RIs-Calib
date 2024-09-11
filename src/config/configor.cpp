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

#include "config/configor.h"
#include "cereal/archives/xml.hpp"
#include "fstream"
#include "spdlog/spdlog.h"
#include "calib/status.hpp"

namespace ns_ris {
    // ------------------------
    // static initialized filed
    // ------------------------
    Configor::DataStream Configor::dataStream = {};
    Configor::Prior Configor::prior = {};
    Configor::Prior::Weight Configor::Prior::weight = {};
    Configor::Preference Configor::preference = {};

    std::map<std::string, std::string> Configor::DataStream::IMUTopics = {};
    std::map<std::string, std::string> Configor::DataStream::RadarTopics = {};
    std::string Configor::DataStream::BagPath = {};
    double Configor::DataStream::BeginTime = {};
    double Configor::DataStream::Duration = {};
    std::string Configor::DataStream::OutputPath = {};

    double Configor::Prior::TimeOffsetPadding = {};
    double Configor::Prior::SplineKnotTimeDistance = {};
    double Configor::Prior::CauchyLossForRadarFactor = {};

    double Configor::Prior::Weight::AcceWeight = {};
    double Configor::Prior::Weight::GyroWeight = {};
    double Configor::Prior::Weight::RadarWeight = {};
    double Configor::Prior::Weight::VelPIMWeight = {};

    bool Configor::Preference::OutputParamInEachIter = {};
    bool Configor::Preference::OutputLMEquationGraph = {};
    bool Configor::Preference::OutputBSplines = {};
    CerealArchiveType::Enum Configor::Preference::OutputDataFormat = CerealArchiveType::Enum::YAML;
    const std::map<CerealArchiveType::Enum, std::string> Configor::Preference::FileExtension = {
            {CerealArchiveType::Enum::YAML,   ".yaml"},
            {CerealArchiveType::Enum::JSON,   ".json"},
            {CerealArchiveType::Enum::XML,    ".xml"},
            {CerealArchiveType::Enum::BINARY, ".bin"}
    };
    bool Configor::Preference::OptTemporalParams = {};
    bool Configor::Preference::OptIntrinsicParams = {};
    int Configor::Preference::ThreadsToUse = {};
    bool Configor::Preference::InitVelocityBspline = {};

    Configor::Configor() = default;

    void Configor::PrintMainFields() {
        std::stringstream streamIMUTopics, streamRadarTopics, streamRadarModels;
        for (const auto &[imuTopic, _]: DataStream::IMUTopics) {
            streamIMUTopics << imuTopic << ' ';
            streamRadarModels << imuTopic << ": " << DataStream::IMUTopics.at(imuTopic) << ' ';
        }
        for (const auto &[radarTopic, _]: DataStream::RadarTopics) {
            streamRadarTopics << radarTopic << ' ';
            streamRadarModels << radarTopic << ": " << DataStream::RadarTopics.at(radarTopic) << ' ';
        }
        std::string IMUTopics = streamIMUTopics.str();
        std::string RadarTopics = streamRadarTopics.str();
        std::string RadarModels = streamRadarModels.str();

#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>35}: {}"
        spdlog::info(
                "main fields of configor:"
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT
                DESC_FORMAT,
                DESC_FIELD(DataStream::BagPath),
                DESC_FIELD(IMUTopics),
                DESC_FIELD(RadarTopics),
                DESC_FIELD(RadarModels),
                DESC_FIELD(DataStream::BeginTime),
                DESC_FIELD(DataStream::Duration),
                DESC_FIELD(DataStream::OutputPath),
                DESC_FIELD(Prior::GravityNorm),
                DESC_FIELD(Prior::TimeOffsetPadding),
                DESC_FIELD(Prior::SplineKnotTimeDistance),
                DESC_FIELD(Prior::CauchyLossForRadarFactor),
                DESC_FIELD(Prior::Weight::AcceWeight),
                DESC_FIELD(Prior::Weight::GyroWeight),
                DESC_FIELD(Prior::Weight::RadarWeight),
                DESC_FIELD(Prior::Weight::VelPIMWeight),
                DESC_FIELD(Preference::OutputParamInEachIter),
                DESC_FIELD(Preference::OutputLMEquationGraph),
                DESC_FIELD(Preference::OutputBSplines),
                "Preference::OutputDataFormat", EnumCast::enumToString(Preference::OutputDataFormat),
                DESC_FIELD(Preference::OptTemporalParams),
                DESC_FIELD(Preference::OptIntrinsicParams),
                DESC_FIELD(Preference::ThreadsToUse),
                DESC_FIELD(Preference::InitVelocityBspline)
        );

#undef DESC_FIELD
#undef DESC_FORMAT
    }

    void Configor::CheckConfigure() {
        if (DataStream::RadarTopics.empty()) {
            throw Status(Status::Flag::ERROR, "the radar topic (i.e., DataStream::RadarTopic) is empty!");
        }
        for (const auto &[radarTopic, _]: DataStream::RadarTopics) {
            if (radarTopic.empty()) {
                throw Status(Status::Flag::ERROR, "empty radar topic exists!");
            }
        }
        if (DataStream::IMUTopics.empty()) {
            throw Status(Status::Flag::ERROR, "the imu topic (i.e., DataStream::IMUTopic) is empty!");
        }
        for (const auto &imuTopic: DataStream::IMUTopics) {
            if (imuTopic.first.empty()) {
                throw Status(Status::Flag::ERROR, "empty imu topic exists!");
            }
        }
        if (DataStream::RadarTopics.empty()) {
            throw Status(Status::Flag::ERROR, "the radar topic (i.e., DataStream::IMUTopic) is empty!");
        }
        for (const auto &radarTopic: DataStream::RadarTopics) {
            if (radarTopic.first.empty()) {
                throw Status(Status::Flag::ERROR, "empty radar topic exists!");
            }
        }
        if (DataStream::BagPath.empty()) {
            throw Status(Status::Flag::ERROR, "the ros bag path (i.e., DataStream::BagPath) is empty!");
        }
        if (DataStream::OutputPath.empty()) {
            throw Status(Status::Flag::ERROR, "the output path (i.e., DataStream::OutputPath) is empty!");
        }
        if (!std::filesystem::exists(DataStream::OutputPath) &&
            !std::filesystem::create_directories(DataStream::OutputPath)) {
            throw Status(Status::Flag::ERROR,
                         "the output path (i.e., DataStream::OutputPath) not exists and create failed!"
            );
        }
        if (Prior::TimeOffsetPadding <= 0.0) {
            throw Status(
                    Status::Flag::ERROR,
                    "the time offset padding (i.e., Prior::TimeOffsetPadding) should be positive!"
            );
        }
        if (Prior::CauchyLossForRadarFactor < 0.0) {
            throw Status(
                    Status::Flag::ERROR, "the cauchy loss for radar factor should be positive!"
            );
        }
    }

    Configor::Ptr Configor::Create() {
        return std::make_shared<Configor>();
    }

    std::string Configor::GetFormatExtension() {
        return Preference::FileExtension.at(Preference::OutputDataFormat);
    }

    bool Configor::LoadConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetInputArchiveVariant(file, archiveType);
        auto configor = Configor::Create();
        SerializeByInputArchiveVariant(archive, archiveType, cereal::make_nvp("Configor", *configor));
        configor->CheckConfigure();
        return true;
    }

    bool Configor::SaveConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("Configor", *this));
        return true;
    }
}
