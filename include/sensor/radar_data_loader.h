// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_RADAR_DATA_LOADER_H
#define RIS_CALIB_RADAR_DATA_LOADER_H

#include "ainstein_radar_msgs/RadarTargetArray.h"
#include "rosbag/message_instance.h"
#include "sensor/radar.h"
#include "util/enum_cast.hpp"

struct EIGEN_ALIGN16 RadarTargetPOSV {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float velocity;    // radial velocity

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        RadarTargetPOSV,
        (float, x, x)(float, y, y)(float, z, z)(float, velocity, velocity)
)
using RadarPOSVCloud = pcl::PointCloud<RadarTargetPOSV>;

struct EIGEN_ALIGN16 RadarTargetPOSIV {
    PCL_ADD_POINT4D;   // quad-word XYZ
    float intensity;   // laser intensity reading
    float velocity;    // radial velocity

    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
        RadarTargetPOSIV,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity)
)
using RadarPOSIVCloud = pcl::PointCloud<RadarTargetPOSIV>;

namespace ns_ris {
    enum class RadarModelType {
        AINSTEIN_RADAR,
        AWR1843BOOST_RAW,
        AWR1843BOOST_CUSTOM,
        POINTCLOUD2_POSV,
        POINTCLOUD2_POSIV
    };

    class RadarDataLoader {
    public:
        using Ptr = std::shared_ptr<RadarDataLoader>;

    protected:
        RadarModelType _radarModel;

    public:
        explicit RadarDataLoader(RadarModelType radarModel);

        virtual RadarTargetArray::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) = 0;

        static RadarDataLoader::Ptr GetLoader(const std::string &radarModelStr);

        [[nodiscard]] RadarModelType GetRadarModel() const;

    protected:
        template<class MsgType>
        void CheckMessage(typename MsgType::ConstPtr msg) {
            if (msg == nullptr) {
                throw std::runtime_error(
                        "message type of some radars was set incorrectly!!! Wrong type: " +
                        std::string(EnumCast::enumToString(GetRadarModel()))
                );
            }
        }
    };

    class AinsteinRadarLoader : public RadarDataLoader {
    public:
        using Ptr = std::shared_ptr<AinsteinRadarLoader>;

    public:
        explicit AinsteinRadarLoader(RadarModelType radarModel);

        static AinsteinRadarLoader::Ptr Create(RadarModelType radarModel);

        RadarTargetArray::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;
    };

    class AWR1843BOOSTRawLoader : public RadarDataLoader {
    public:
        using Ptr = std::shared_ptr<AWR1843BOOSTRawLoader>;

    public:
        explicit AWR1843BOOSTRawLoader(RadarModelType radarModel);

        static AWR1843BOOSTRawLoader::Ptr Create(RadarModelType radarModel);

        RadarTargetArray::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;
    };

    class PointCloud2POSVLoader : public RadarDataLoader {
    public:
        using Ptr = std::shared_ptr<PointCloud2POSVLoader>;

    public:
        explicit PointCloud2POSVLoader(RadarModelType radarModel);

        static PointCloud2POSVLoader::Ptr Create(RadarModelType radarModel);

        RadarTargetArray::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;
    };

    class PointCloud2POSIVLoader : public RadarDataLoader {
    public:
        using Ptr = std::shared_ptr<PointCloud2POSIVLoader>;

    public:
        explicit PointCloud2POSIVLoader(RadarModelType radarModel);

        static PointCloud2POSIVLoader::Ptr Create(RadarModelType radarModel);

        RadarTargetArray::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;
    };


    class AWR1843BOOSTCustomLoader : public RadarDataLoader {
    public:
        using Ptr = std::shared_ptr<AWR1843BOOSTCustomLoader>;

    public:
        explicit AWR1843BOOSTCustomLoader(RadarModelType radarModel);

        static AWR1843BOOSTCustomLoader::Ptr Create(RadarModelType radarModel);

        RadarTargetArray::Ptr UnpackScan(const rosbag::MessageInstance &msgInstance) override;
    };
}

#endif //RIS_CALIB_RADAR_DATA_LOADER_H
