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

#include "nofree/bag_merge.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ris_calib_merge_node");

    try {
        // auto configor = ns_ris::MergeConfigor();
        // configor.SaveConfigure(
        //         "/home/csl/ros_ws/RIs-Calib/src/ris_calib/config/config-merge11.yaml",
        //         ns_ris::CerealArchiveType::Enum::YAML
        // );
        // std::cin.get();

        // load settings
        std::string configPath;
        if (!ros::param::get("/ris_calib_merge_node/config_path", configPath)) {
            throw ns_ris::Status(
                    ns_ris::Status::Flag::CRITICAL,
                    "the configure path couldn't obtained from ros param '/ris_calib_merge_node/config_path'."
            );
        }
        spdlog::info("loading configure from json file '{}'...", configPath);
        auto mergeConfigor = ns_ris::MergeConfigor::LoadConfigure(configPath, ns_ris::CerealArchiveType::Enum::YAML);
        if (!mergeConfigor) {
            throw ns_ris::Status(ns_ris::Status::Flag::CRITICAL, "load configure file failed!");
        } else {
            mergeConfigor->PrintMainFields();
        }

        ns_ris::BagMerger::Create(mergeConfigor)->Process();

    } catch (const ns_ris::Status &status) {
        // if error happened, print it
        switch (status.flag) {
            case ns_ris::Status::Flag::FINE:
                // this case usually won't happen
                spdlog::info(status.what);
                break;
            case ns_ris::Status::Flag::WARNING:
                spdlog::warn(status.what);
                break;
            case ns_ris::Status::Flag::ERROR:
                spdlog::error(status.what);
                break;
            case ns_ris::Status::Flag::CRITICAL:
                spdlog::critical(status.what);
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        spdlog::critical(e.what());
    }

    ros::shutdown();
    return 0;
}