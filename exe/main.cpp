// Copyright (c) 2023. Created on 7/7/23 1:28 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "calib/calib_solver.h"
#include "ros/ros.h"

// config the 'spdlog' log pattern
void ConfigSpdlog() {
    // [log type]-[thread]-[time] message
    spdlog::set_pattern("%^[%L]%$-[%t]-[%H:%M:%S.%e] %v");

    // set log level
    spdlog::set_level(spdlog::level::debug);
}

void PrintLibInfo() {
    // ns_pretab::PrettyTable tab;
    // tab.addRowGrids(0, 1, 0, 2, ns_pretab::Align::CENTER, "");
    // tab.addGrid(1, 0, "RIs-Calib");
    // tab.addGrid(1, 1, "https://github.com/Unsigned-Long/RIs-Calib.git");
    // tab.addGrid(2, 0, "Author");
    // tab.addGrid(2, 1, "Shuolong Chen");
    // tab.addGrid(3, 0, "E-Mail");
    // tab.addGrid(3, 1, "shlchen@whu.edu.cn");
    // std::cout << tab << std::endl;
    std::cout << "+----------------------------------------------------------------------------+\n"
                 "| _|_|_|    _|_|_|                        _|_|_|            _|  _|  _|       |\n"
                 "| _|    _|    _|      _|_|_|            _|          _|_|_|  _|      _|_|_|   |\n"
                 "| _|_|_|      _|    _|_|      _|_|_|_|  _|        _|    _|  _|  _|  _|    _| |\n"
                 "| _|    _|    _|        _|_|            _|        _|    _|  _|  _|  _|    _| |\n"
                 "| _|    _|  _|_|_|  _|_|_|                _|_|_|    _|_|_|  _|  _|  _|_|_|   |\n"
                 "+-----------+----------------------------------------------------------------+\n"
                 "| RIs-Calib | https://github.com/Unsigned-Long/RIs-Calib.git                 |\n"
                 "+-----------+----------------------------------------------------------------+\n"
                 "| Author    | Shuolong Chen                                                  |\n"
                 "+-----------+----------------------------------------------------------------+\n"
                 "| E-Mail    | shlchen@whu.edu.cn                                             |\n"
                 "+-----------+----------------------------------------------------------------+" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ris_calib_prog_node");
    try {
        ConfigSpdlog();

        PrintLibInfo();

        // auto configor = ns_ris::Configor();
        // configor.SaveConfigure("/home/csl/ros_ws/RIs-Calib/src/ris_calib/config/config.yaml");
        // std::cin.get();

        // load settings
        std::string configPath;
        if (!ros::param::get("/ris_calib_prog_node/config_path", configPath)) {
            throw ns_ris::Status(
                    ns_ris::Status::Flag::CRITICAL,
                    "the configure path couldn't obtained from ros param '/ris_calib_prog_node/config_path'."
            );
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        if (!ns_ris::Configor::LoadConfigure(configPath)) {
            throw ns_ris::Status(ns_ris::Status::Flag::CRITICAL, "load configure file failed!");
        } else {
            ns_ris::Configor::PrintMainFields();
        }

        // create parameter manager
        auto paramManager = ns_ris::CalibParamManager::Create();
        // initialize and show parameter manager
        paramManager->InitializeParametersFromConfigor();
        paramManager->ShowParamStatus();
        // create data manager
        auto dataManager = ns_ris::CalibDataManager::Create();

        // pass parameter manager and data manager to solver for solving
        auto solver = ns_ris::CalibSolver::Create(dataManager, paramManager);
        solver->Process();

        // solve finished, save calibration results (file type: JSON | YAML | XML | BINARY)
        paramManager->Save(
                ns_ris::Configor::DataStream::OutputPath + "/param" + ns_ris::Configor::GetFormatExtension(),
                ns_ris::Configor::Preference::OutputDataFormat
        );
        // aligned st parameters to the first imu
        if (ns_ris::Configor::DataStream::IMUTopics.size() > 1) {
            paramManager->AlignParamToNewSensor(ns_ris::Configor::DataStream::IMUTopics.cbegin()->first)->Save(
                    ns_ris::Configor::DataStream::OutputPath + "/param_align_to_fir_imu" +
                    ns_ris::Configor::GetFormatExtension(), ns_ris::Configor::Preference::OutputDataFormat
            );
        }

        // -------------
        // visualization
        // -------------
        // solver->VisualizationSensorSuite();
        // solver->VisualizationBSplines();
        // solver->Visualization();

        // save splines
        if (ns_ris::Configor::Preference::OutputBSplines) {
            solver->SaveBSplines(100);
        }

        spdlog::info("Solving is finished, the spatiotemporal parameters are output in the console, "
                     "the results (sensor suites and B-splines) are also displayed in the pangolin-derived window.");

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