// Copyright (c) 2023. Created on 9/20/23 12:18 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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