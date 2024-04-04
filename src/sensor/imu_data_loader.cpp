// Copyright (c) 2023. Created on 9/20/23 8:06 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "sensor/imu_data_loader.h"
#include "util/enum_cast.hpp"
#include "calib/status.hpp"
#include "spdlog/fmt/fmt.h"

namespace ns_ris {

    IMUDataLoader::IMUDataLoader(IMUModelType imuModel) : _imuModel(imuModel) {}

    IMUDataLoader::Ptr IMUDataLoader::GetLoader(const std::string &imuModelStr) {
        // try extract radar model
        IMUModelType imuModel;
        try {
            imuModel = EnumCast::stringToEnum<IMUModelType>(imuModelStr);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported IMU Type: '{}'. "
                            "Currently supported IMU types are: \n"
                            "(1) SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
                            "(2)    SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git\n"
                            "...\n"
                            "If you need to use other IMU types, "
                            "please 'Issues' us on the profile of the github repository.",
                            imuModelStr
                    )
            );
        }
        IMUDataLoader::Ptr imuDataLoader;
        switch (imuModel) {
            case IMUModelType::SENSOR_IMU:
                imuDataLoader = SensorIMULoader::Create(imuModel);
                break;
            case IMUModelType::SBG_IMU:
                imuDataLoader = SbgIMULoader::Create(imuModel);
                break;
        }
        return imuDataLoader;
    }

    IMUModelType IMUDataLoader::GetIMUModel() const {
        return _imuModel;
    }

    SensorIMULoader::SensorIMULoader(IMUModelType imuModel) : IMUDataLoader(imuModel) {}

    SensorIMULoader::Ptr SensorIMULoader::Create(IMUModelType imuModel) {
        return std::make_shared<SensorIMULoader>(imuModel);
    }

    IMUFrame::Ptr SensorIMULoader::UnpackFrame(const rosbag::MessageInstance &msgInstance) {
        // imu data item
        sensor_msgs::ImuConstPtr msg = msgInstance.instantiate<sensor_msgs::Imu>();

        CheckMessage<sensor_msgs::Imu>(msg);

        auto acce = Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
        );
        auto gyro = Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
        );

        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }

    SbgIMULoader::SbgIMULoader(IMUModelType imuModel) : IMUDataLoader(imuModel) {}

    SbgIMULoader::Ptr SbgIMULoader::Create(IMUModelType imuModel) {
        return std::make_shared<SbgIMULoader>(imuModel);
    }

    IMUFrame::Ptr SbgIMULoader::UnpackFrame(const rosbag::MessageInstance &msgInstance) {
        // imu data item
        sbg_driver::SbgImuDataConstPtr msg = msgInstance.instantiate<sbg_driver::SbgImuData>();

        CheckMessage<sbg_driver::SbgImuData>(msg);

        auto acce = Eigen::Vector3d(
                msg->accel.x,
                msg->accel.y,
                msg->accel.z
        );
        auto gyro = Eigen::Vector3d(
                msg->gyro.x,
                msg->gyro.y,
                msg->gyro.z
        );

        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }
}