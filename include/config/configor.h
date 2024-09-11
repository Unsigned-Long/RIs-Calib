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

#ifndef RIS_CALIB_CONFIGOR_H
#define RIS_CALIB_CONFIGOR_H

#include "memory"
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/set.hpp"
#include "util/utils.hpp"
#include "util/enum_cast.hpp"

namespace ns_ris {
    struct Configor {
    public:
        using Ptr = std::shared_ptr<Configor>;

    public:

        static struct DataStream {
            static std::map<std::string, std::string> IMUTopics;
            // [radar topic, model]
            static std::map<std::string, std::string> RadarTopics;

            static std::string BagPath;
            static double BeginTime;
            static double Duration;

            static std::string OutputPath;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(IMUTopics), CEREAL_NVP(RadarTopics),
                        CEREAL_NVP(BagPath), CEREAL_NVP(BeginTime), CEREAL_NVP(Duration),
                        CEREAL_NVP(OutputPath)
                );
            }
        } dataStream;

        static struct Prior {
            static constexpr double GravityNorm = 9.797;
            static constexpr int SplineOrder = 4;
            static double TimeOffsetPadding;
            static double SplineKnotTimeDistance;
            static double CauchyLossForRadarFactor;

            static struct Weight {
                static double AcceWeight;
                static double GyroWeight;
                static double RadarWeight;
                static double VelPIMWeight;

            public:
                template<class Archive>
                void serialize(Archive &ar) {
                    ar(CEREAL_NVP(AcceWeight), CEREAL_NVP(GyroWeight),
                       CEREAL_NVP(RadarWeight), CEREAL_NVP(VelPIMWeight));
                }
            } weight;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(TimeOffsetPadding), CEREAL_NVP(SplineKnotTimeDistance),
                   cereal::make_nvp("Weight", weight), CEREAL_NVP(CauchyLossForRadarFactor));
            }
        } prior;

        static struct Preference {

            static bool OutputParamInEachIter;
            static bool OutputLMEquationGraph;
            static bool OutputBSplines;
            static CerealArchiveType::Enum OutputDataFormat;
            const static std::map<CerealArchiveType::Enum, std::string> FileExtension;
            static bool OptTemporalParams;
            static bool OptIntrinsicParams;
            static int ThreadsToUse;
            static bool InitVelocityBspline;

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(
                        CEREAL_NVP(OutputParamInEachIter),
                        CEREAL_NVP(OutputLMEquationGraph),
                        CEREAL_NVP(OutputBSplines),
                        CEREAL_NVP(OutputDataFormat),
                        CEREAL_NVP(OptTemporalParams),
                        CEREAL_NVP(OptIntrinsicParams),
                        CEREAL_NVP(ThreadsToUse),
                        CEREAL_NVP(InitVelocityBspline)
                );
            }
        } preference;

    public:
        Configor();

        static Ptr Create();

        // load configure information from file
        template<class CerealArchiveType=CerealArchiveType::YAML>
        static bool LoadConfigure(const std::string &filename) {
            std::ifstream file(filename);
            auto archive = GetInputArchive<CerealArchiveType>(file);
            auto configor = Configor::Create();
            (*archive)(cereal::make_nvp("Configor", *configor));
            configor->CheckConfigure();
            return true;
        }

        // save configure information to file
        template<class CerealArchiveType=CerealArchiveType::YAML>
        bool SaveConfigure(const std::string &filename) {
            std::ofstream file(filename);
            auto archive = GetOutputArchive<CerealArchiveType>(file);
            (*archive)(cereal::make_nvp("Configor", *this));
            return true;
        }

        // load configure information from file
        static bool LoadConfigure(const std::string &filename, CerealArchiveType::Enum archiveType);

        // save configure information to file
        bool SaveConfigure(const std::string &filename, CerealArchiveType::Enum archiveType);

        // print the main fields
        static void PrintMainFields();

        static std::string GetFormatExtension();

    protected:
        // check the input configure
        static void CheckConfigure();

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("DataStream", dataStream), cereal::make_nvp("Prior", prior),
               cereal::make_nvp("Preference", preference));
        }
    };
}

#endif //RIS_CALIB_CONFIGOR_H
