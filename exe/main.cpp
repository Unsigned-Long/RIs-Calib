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