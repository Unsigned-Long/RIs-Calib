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

#include "sensor/radar_data_loader.h"
#include "ti_mmwave_rospkg/RadarScan.h"
#include "ti_mmwave_rospkg/RadarScanCustom.h"
#include "util/enum_cast.hpp"
#include "calib/status.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

namespace ns_ris {

    RadarDataLoader::RadarDataLoader(RadarModelType radarModel) : _radarModel(radarModel) {}

    RadarDataLoader::Ptr RadarDataLoader::GetLoader(const std::string &radarModelStr) {
        // try extract radar model
        RadarModelType radarModel;
        try {
            radarModel = EnumCast::stringToEnum<RadarModelType>(radarModelStr);
        } catch (...) {
            throw Status(
                    Status::Flag::WARNING,
                    fmt::format(
                            "Unsupported Radar Type: '{}'. "
                            "Currently supported radar types are: \n"
                            "(1)      AINSTEIN_RADAR: https://github.com/AinsteinAI/ainstein_radar.git\n"
                            "(2)    AWR1843BOOST_RAW: https://github.com/Unsigned-Long/ti_mmwave_rospkg.git\n"
                            "(2) AWR1843BOOST_CUSTOM: https://github.com/Unsigned-Long/ti_mmwave_rospkg.git\n"
                            "(3)    POINTCLOUD2_POSV: 'sensor_msgs/PointCloud2' with point format: [x, y, z, velocity]\n"
                            "(4)   POINTCLOUD2_POSIV: 'sensor_msgs/PointCloud2' with point format: [x, y, z, intensity, velocity]\n"
                            "...\n"
                            "If you need to use other radar types, "
                            "please 'Issues' us on the profile of the github repository.",
                            radarModelStr
                    )
            );
        }
        RadarDataLoader::Ptr radarDataLoader;
        switch (radarModel) {
            case RadarModelType::AINSTEIN_RADAR:
                radarDataLoader = AinsteinRadarLoader::Create(radarModel);
                break;
            case RadarModelType::AWR1843BOOST_RAW:
                radarDataLoader = AWR1843BOOSTRawLoader::Create(radarModel);
                break;
            case RadarModelType::POINTCLOUD2_POSV:
                radarDataLoader = PointCloud2POSVLoader::Create(radarModel);
                break;
            case RadarModelType::POINTCLOUD2_POSIV:
                radarDataLoader = PointCloud2POSIVLoader::Create(radarModel);
                break;
            case RadarModelType::AWR1843BOOST_CUSTOM:
                radarDataLoader = AWR1843BOOSTCustomLoader::Create(radarModel);
                break;
        }
        return radarDataLoader;
    }

    RadarModelType RadarDataLoader::GetRadarModel() const {
        return _radarModel;
    }

    AinsteinRadarLoader::AinsteinRadarLoader(RadarModelType radarModel) : RadarDataLoader(radarModel) {}

    AinsteinRadarLoader::Ptr AinsteinRadarLoader::Create(RadarModelType radarModel) {
        return std::make_shared<AinsteinRadarLoader>(radarModel);
    }

    RadarTargetArray::Ptr AinsteinRadarLoader::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        ainstein_radar_msgs::RadarTargetArray::ConstPtr msg =
                msgInstance.instantiate<ainstein_radar_msgs::RadarTargetArray>();

        CheckMessage<ainstein_radar_msgs::RadarTargetArray>(msg);

        std::vector<RadarTarget::Ptr> targets(msg->targets.size());

        for (int i = 0; i < static_cast<int>(msg->targets.size()); ++i) {
            const auto &tar = msg->targets.at(i);
            targets.at(i) = RadarTarget::Create(
                    msg->header.stamp.toSec(), {tar.range, tar.azimuth, tar.elevation, tar.speed}
            );
        }

        return RadarTargetArray::Create(msg->header.stamp.toSec(), targets);
    }

    AWR1843BOOSTRawLoader::AWR1843BOOSTRawLoader(RadarModelType radarModel) : RadarDataLoader(radarModel) {}

    AWR1843BOOSTRawLoader::Ptr AWR1843BOOSTRawLoader::Create(RadarModelType radarModel) {
        return std::make_shared<AWR1843BOOSTRawLoader>(radarModel);
    }

    RadarTargetArray::Ptr AWR1843BOOSTRawLoader::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        // for ti mm wave radar, every event saved singly
        ti_mmwave_rospkg::RadarScan::ConstPtr msg = msgInstance.instantiate<ti_mmwave_rospkg::RadarScan>();

        CheckMessage<ti_mmwave_rospkg::RadarScan>(msg);

        auto target = RadarTarget::Create(
                msg->header.stamp.toSec(), {msg->x, msg->y, msg->z}, msg->velocity
        );

        return RadarTargetArray::Create(msg->header.stamp.toSec(), {target});
    }

    PointCloud2POSVLoader::PointCloud2POSVLoader(RadarModelType radarModel) : RadarDataLoader(radarModel) {}

    PointCloud2POSVLoader::Ptr PointCloud2POSVLoader::Create(RadarModelType radarModel) {
        return std::make_shared<PointCloud2POSVLoader>(radarModel);
    }

    RadarTargetArray::Ptr PointCloud2POSVLoader::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        sensor_msgs::PointCloud2::ConstPtr msg = msgInstance.instantiate<sensor_msgs::PointCloud2>();

        CheckMessage<sensor_msgs::PointCloud2>(msg);

        RadarPOSVCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) || std::isnan(tar.velocity)) { continue; }
            // the timestamp should be obtained from header, rather than from instance
            targets.push_back(
                    RadarTarget::Create(msgInstance.getTime().toSec(), {tar.x, tar.y, tar.z}, tar.velocity));
        }

        return RadarTargetArray::Create(msgInstance.getTime().toSec(), targets);
    }

    PointCloud2POSIVLoader::PointCloud2POSIVLoader(RadarModelType radarModel) : RadarDataLoader(radarModel) {}

    PointCloud2POSIVLoader::Ptr PointCloud2POSIVLoader::Create(RadarModelType radarModel) {
        return std::make_shared<PointCloud2POSIVLoader>(radarModel);
    }

    RadarTargetArray::Ptr PointCloud2POSIVLoader::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        sensor_msgs::PointCloud2::ConstPtr msg = msgInstance.instantiate<sensor_msgs::PointCloud2>();

        CheckMessage<sensor_msgs::PointCloud2>(msg);

        RadarPOSIVCloud radarTargets;
        pcl::fromROSMsg(*msg, radarTargets);

        std::vector<RadarTarget::Ptr> targets;
        targets.reserve(radarTargets.size());

        for (const auto &tar: radarTargets) {
            if (std::isnan(tar.x) || std::isnan(tar.y) || std::isnan(tar.z) || std::isnan(tar.velocity)) { continue; }
            // the timestamp should be obtained from header, rather than from instance
            targets.push_back(
                    RadarTarget::Create(msgInstance.getTime().toSec(), {tar.x, tar.y, tar.z}, tar.velocity));
        }

        return RadarTargetArray::Create(msgInstance.getTime().toSec(), targets);
    }

    AWR1843BOOSTCustomLoader::AWR1843BOOSTCustomLoader(RadarModelType radarModel) : RadarDataLoader(radarModel) {}

    AWR1843BOOSTCustomLoader::Ptr AWR1843BOOSTCustomLoader::Create(RadarModelType radarModel) {
        return std::make_shared<AWR1843BOOSTCustomLoader>(radarModel);
    }

    RadarTargetArray::Ptr AWR1843BOOSTCustomLoader::UnpackScan(const rosbag::MessageInstance &msgInstance) {
        // for ti mm wave radar, every event saved singly
        ti_mmwave_rospkg::RadarScanCustom::ConstPtr msg = msgInstance.instantiate<ti_mmwave_rospkg::RadarScanCustom>();

        CheckMessage<ti_mmwave_rospkg::RadarScanCustom>(msg);

        auto target = RadarTarget::Create(
                msg->header.stamp.toSec(), {msg->x, msg->y, msg->z}, msg->velocity
        );

        return RadarTargetArray::Create(msg->header.stamp.toSec(), {target});
    }
}