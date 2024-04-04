// Copyright (c) 2023. Created on 9/20/23 8:06 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_IMU_DATA_LOADER_H
#define RIS_CALIB_IMU_DATA_LOADER_H

#include "sensor_msgs/Imu.h"
#include "sbg_driver/SbgImuData.h"
#include "rosbag/message_instance.h"
#include "sensor/imu.h"
#include "util/enum_cast.hpp"

namespace ns_ris {
    enum class IMUModelType {
        SENSOR_IMU,
        SBG_IMU
    };


    class IMUDataLoader {
    public:
        using Ptr = std::shared_ptr<IMUDataLoader>;

    protected:
        IMUModelType _imuModel;

    public:
        explicit IMUDataLoader(IMUModelType imuModel);

        virtual IMUFrame::Ptr UnpackFrame(const rosbag::MessageInstance &msgInstance) = 0;

        static IMUDataLoader::Ptr GetLoader(const std::string &imuModelStr);

        [[nodiscard]] IMUModelType GetIMUModel() const;

    protected:
        template<class MsgType>
        void CheckMessage(typename MsgType::ConstPtr msg) {
            if (msg == nullptr) {
                throw std::runtime_error(
                        "message type of some IMUs was set incorrectly!!! Wrong type: " +
                        std::string(EnumCast::enumToString(GetIMUModel()))
                );
            }
        }
    };

    class SensorIMULoader : public IMUDataLoader {
    public:
        using Ptr = std::shared_ptr<SensorIMULoader>;

    public:
        explicit SensorIMULoader(IMUModelType imuModel);

        static SensorIMULoader::Ptr Create(IMUModelType imuModel);

        IMUFrame::Ptr UnpackFrame(const rosbag::MessageInstance &msgInstance) override;
    };

    class SbgIMULoader : public IMUDataLoader {
    public:
        using Ptr = std::shared_ptr<SbgIMULoader>;

    public:
        explicit SbgIMULoader(IMUModelType imuModel);

        static SbgIMULoader::Ptr Create(IMUModelType imuModel);

        IMUFrame::Ptr UnpackFrame(const rosbag::MessageInstance &msgInstance) override;
    };
}


#endif //RIS_CALIB_IMU_DATA_LOADER_H
