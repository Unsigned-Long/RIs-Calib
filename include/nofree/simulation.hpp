// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_SIMULATION_HPP
#define RIS_CALIB_SIMULATION_HPP

#include <utility>

#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "ainstein_radar_msgs/RadarTargetArray.h"
#include "ctraj/core/trajectory.h"

namespace ns_ris {
    struct MultiSensorSimulator {
    protected:
        ns_ctraj::Trajectory<4>::Ptr _traj;

        const std::string _savePath;
        Eigen::Vector3d _gravity;

    public:
        explicit MultiSensorSimulator(std::string savePath) : _savePath(std::move(savePath)) {
            // trajectory
            // _traj = ns_ctraj::SimuEightShapeMotion<4>(5.0, 4.0, 0.5).GetTrajectory();
            _traj = ns_ctraj::SimuDrunkardMotion<4>({0.0, 0.0, 0.0}, 0.5, 60.0).GetTrajectory();
            _gravity = {0.0, 0.0, -Configor::Prior::GravityNorm};
            {
                // visualization
                ns_ctraj::Viewer viewer;
                _traj->Visualization(viewer);
                viewer.RunInSingleThread();
            }
        }

        void Simulate() {
            auto bag = std::make_unique<rosbag::Bag>();
            bag->open(_savePath + "/radar_imu.bag", rosbag::BagMode::Write);
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> tRand(-0.01, 0.01);

            auto calibParam = CalibParamManager::Create();
            calibParam->GRAVITY = _gravity;

            for (int i = 0; i < 3; ++i) {
                {
                    auto topic = "/simu_imu" + std::to_string(i);
                    spdlog::info("simulate '{}'...", topic);

                    auto bias = GeneratePoseBias();
                    double timeOffset = tRand(engine);

                    SimulateIMU(bag, topic, bias, _gravity, 400, timeOffset);

                    // record
                    calibParam->EXTRI.SO3_BiToBc.insert({topic, bias.so3()});
                    calibParam->EXTRI.POS_BiInBc.insert({topic, bias.translation()});
                    calibParam->TEMPORAL.TIME_OFFSET_BiToBc.insert({topic, timeOffset});
                    calibParam->INTRI.IMU.insert({topic, {}});
                    calibParam->INTRI.IMU.at(topic).Clear();
                }
                {
                    auto topic = "/simu_radar" + std::to_string(i);
                    spdlog::info("simulate '{}'...", topic);

                    auto bias = GeneratePoseBias();
                    double timeOffset = tRand(engine);

                    SimulateRadar(bag, topic, bias, 10, timeOffset);

                    // record
                    calibParam->EXTRI.SO3_RjToBc.insert({topic, bias.so3()});
                    calibParam->EXTRI.POS_RjInBc.insert({topic, bias.translation()});
                    calibParam->TEMPORAL.TIME_OFFSET_RjToBc.insert({topic, timeOffset});
                }
            }
            bag->close();
            calibParam->Save(_savePath + "/truth.json");

            auto alignedParam = calibParam->AlignParamToNewSensor(
                    calibParam->EXTRI.SE3_BiToBc("/simu_imu0").inverse(),
                    -calibParam->TEMPORAL.TIME_OFFSET_BiToBc.at("/simu_imu0")
            );
            alignedParam->Save(_savePath + "/truth_align_to_imu0.json");
            alignedParam->ShowParamStatus();

            ns_viewer::Viewer viewer;
            calibParam->VisualizationSensors(viewer);
            viewer.RunInSingleThread();
        }

    protected:

        static Sophus::SE3d GeneratePoseBias() {
            constexpr double DEG_TO_RAD = M_PI / 180.0;
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> aRand(-180, 180), pRand(-0.5, 0.5);
            auto a1 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
            auto a2 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
            auto a3 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
            return {(a1 * a2 * a3).toRotationMatrix(), Eigen::Vector3d(pRand(engine), pRand(engine), pRand(engine))};
        }

        void SimulateIMU(const std::unique_ptr<rosbag::Bag> &bag, const std::string &imuTopic,
                         const Sophus::SE3d &SE3_BiToRef, const Eigen::Vector3d &gravityInRef0,
                         int hz, double timeOffset) {
            // imu data
            auto imuMes = this->_traj->ComputeIMUMeasurement(
                    gravityInRef0, SE3_BiToRef, 1.0 / static_cast<double>(hz)
            );
            // write imu msgs
            spdlog::info("insert imu messages for topic '{}', hz: '{}'", imuTopic, hz);
            for (const auto &frame: imuMes) {
                if (frame->GetTimestamp() - timeOffset < ros::TIME_MIN.toSec()) { continue; }

                sensor_msgs::Imu imuMsg;
                imuMsg.header.stamp = ros::Time(frame->GetTimestamp() - timeOffset);
                imuMsg.header.frame_id = "imu";

                imuMsg.angular_velocity.x = frame->GetGyro()(0);
                imuMsg.angular_velocity.y = frame->GetGyro()(1);
                imuMsg.angular_velocity.z = frame->GetGyro()(2);

                imuMsg.linear_acceleration.x = frame->GetAcce()(0);
                imuMsg.linear_acceleration.y = frame->GetAcce()(1);
                imuMsg.linear_acceleration.z = frame->GetAcce()(2);

                bag->write(imuTopic, imuMsg.header.stamp, imuMsg);
            }
        }

        void SimulateRadar(const std::unique_ptr<rosbag::Bag> &bag, const std::string &radarTopic,
                           const Sophus::SE3d &SE3_RjToRef,
                           int hz, double timeOffset) {
            // radar data
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> ux(-5.0, 5.0), uy(-4.0, 4.0), uz(-1.0, 1.0);
            // simu targets
            Eigen::aligned_vector<Eigen::Vector3d> targets;
            for (int i = 0; i < 100; ++i) {
                Eigen::Vector3d target(ux(engine), uy(engine), uz(engine));
                targets.push_back(target);
            }
            // simu mes
            std::vector<RadarTargetArray::Ptr> radarMes;
            for (const auto &item: _traj->Sampling(1.0 / static_cast<double>(hz))) {
                double timeByIMU = item.timeStamp;
                std::vector<RadarTarget::Ptr> array;
                for (const auto &target: targets) {
                    auto obv = RadarTarget::Create(
                            timeByIMU - timeOffset,
                            // attention: here is 'timeByIMU', not 'timeByIMU - TIME_OFFSET_RtoB'
                            _traj->RadarStaticMeasurement(timeByIMU, target, SE3_RjToRef)
                    );
                    array.push_back(obv);
                }
                radarMes.push_back(RadarTargetArray::Create(timeByIMU - timeOffset, array));
            }

            // write radar msgs
            spdlog::info("insert radar messages for topic '{}', hz: '{}'", radarTopic, hz);
            for (const auto &array: radarMes) {
                if (array->GetTimestamp() < ros::TIME_MIN.toSec()) { continue; }

                ainstein_radar_msgs::RadarTargetArray tarAry;
                tarAry.header.stamp = ros::Time(array->GetTimestamp());
                tarAry.header.frame_id = "radar";

                for (const auto &frame: array->GetTargets()) {
                    Eigen::Vector3d rtp = frame->GetTargetRTP();

                    ainstein_radar_msgs::RadarTarget tar;
                    tar.range = rtp(0);
                    tar.azimuth = rtp(1);
                    tar.elevation = rtp(2);
                    tar.speed = frame->GetRadialVelocity();

                    tarAry.targets.push_back(tar);
                }

                bag->write(radarTopic, tarAry.header.stamp, tarAry);
            }
        }
    };
}

#endif //RIS_CALIB_SIMULATION_HPP
