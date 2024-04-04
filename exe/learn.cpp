// Copyright (c) 2023. Created on 7/7/23 1:28 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "calib/calib_param_manager.h"
#include "calib/status.hpp"
#include "ros/ros.h"
#include "nofree/radar_model.hpp"
#include "nofree/simulation.hpp"
#include "nofree/equation_verify.hpp"
#include "nofree/error_statistics.hpp"

// config the 'spdlog' log pattern
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::debug);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ris_calib_learn_node");
    try {
        ConfigSpdlog();

        // load settings
        std::string configPath;
        if (!ros::param::get("/ris_calib_learn_node/config_path", configPath)) {
            throw ns_ris::Status(
                    ns_ris::Status::Flag::CRITICAL,
                    "the configure path couldn't obtained from ros param '/ris_calib_learn_node/config_path'."
            );
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        // ns_ris::RadarModelLearner::Learn();
        // ns_ris::RIsInitLearner::Learn();
        // ns_ris::MultiSensorSimulator("/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu4").Simulate();
        // ns_ris::EquationVerify::Equation2();
        // ns_ris::CalibParamManager::Load(
        //         "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/truth_align_to_imu0.json"
        // )->ShowParamStatus();
        // ns_ris::CalibParamManager::Load(
        //         "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/param_align_to_fir_imu.json"
        // )->ShowParamStatus();
        // ns_ris::ErrorStatistic::SaveSTParamsAsRawJson(ns_ris::ErrorStatistic::PerformForParamsIter(
        //         "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/params_iter",
        //         "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/truth_align_to_imu0.json"
        // ), "/home/csl/paper_create/RIs-Calib-Paper/manuscript/img/simulation/opt_process/errors.json");
        auto viewer = ns_viewer::Viewer::Load(
                "/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/sensor.view");
        viewer->GetConfigor().WithScreenShotSaveDir("/home/csl/ros_ws/RIs-Calib/docs/img");
        viewer->RunInSingleThread();

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