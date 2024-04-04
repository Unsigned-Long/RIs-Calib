// Copyright (c) 2023. Created on 7/7/23 1:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#include "calib/calib_data_manager.h"
#include "rosbag/view.h"
#include "spdlog/spdlog.h"
#include "sensor_msgs/Imu.h"
#include "sensor/radar_data_loader.h"
#include "sensor/imu_data_loader.h"

namespace ns_ris {

    CalibDataManager::CalibDataManager() {
        LoadCalibData();
        AdjustCalibDataSequence();
        AlignTimestamp();
    }

    CalibDataManager::Ptr CalibDataManager::Create() {
        return std::make_shared<CalibDataManager>();
    }

    void CalibDataManager::LoadCalibData() {
        spdlog::info("load calibration data...");

        // open the ros bag
        auto bag = std::make_unique<rosbag::Bag>();
        if (!std::filesystem::exists(Configor::DataStream::BagPath)) {
            spdlog::error("the ros bag path '{}' is invalid!", Configor::DataStream::BagPath);
        } else {
            bag->open(Configor::DataStream::BagPath, rosbag::BagMode::Read);
        }

        auto view = rosbag::View();

        // using a temp view to check the time range of the source ros bag
        auto viewTemp = rosbag::View();

        std::vector<std::string> topicsToQuery;
        // add topics to vector
        auto IMUTopics = Configor::DataStream::IMUTopics;
        auto RadarTopics = Configor::DataStream::RadarTopics;
        for (const auto &[item, _]: IMUTopics) { topicsToQuery.push_back(item); }
        for (const auto &[item, _]: RadarTopics) { topicsToQuery.push_back(item); }

        viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
        auto begTime = viewTemp.getBeginTime();
        auto endTime = viewTemp.getEndTime();
        spdlog::info("source data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(), endTime.toSec());

        // adjust the data time range
        if (Configor::DataStream::BeginTime > 0.0) {
            begTime += ros::Duration(Configor::DataStream::BeginTime);
            if (begTime > endTime) {
                spdlog::warn(
                        "begin time '{:.5f}' is out of the bag's data range, set begin time to '{:.5f}'.",
                        begTime.toSec(), viewTemp.getBeginTime().toSec()
                );
                begTime = viewTemp.getBeginTime();
            }
        }
        if (Configor::DataStream::Duration > 0.0) {
            endTime = begTime + ros::Duration(Configor::DataStream::Duration);
            if (endTime > viewTemp.getEndTime()) {
                spdlog::warn(
                        "end time '{:.5f}' is out of the bag's data range, set end time to '{:.5f}'.",
                        endTime.toSec(), viewTemp.getEndTime().toSec()
                );
                endTime = viewTemp.getEndTime();
            }
        }
        spdlog::info("expect data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(), endTime.toSec());

        view.addQuery(*bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime);

        // create radar data loader
        std::map<std::string, RadarDataLoader::Ptr> radarDataLoaders;
        // create IMU data loader
        std::map<std::string, IMUDataLoader::Ptr> imuDataLoaders;

        // get radar type enum from the string
        for (const auto &[radarTopic, radarModelStr]: Configor::DataStream::RadarTopics) {
            radarDataLoaders.insert({radarTopic, RadarDataLoader::GetLoader(radarModelStr)});
        }
        for (const auto &[imuTopic, imuModelStr]: Configor::DataStream::IMUTopics) {
            imuDataLoaders.insert({imuTopic, IMUDataLoader::GetLoader(imuModelStr)});
        }

        // read raw data
        for (const auto &item: view) {
            const std::string &topic = item.getTopic();
            if (IMUTopics.cend() != IMUTopics.find(topic)) {

                auto msg = imuDataLoaders.at(topic)->UnpackFrame(item);
                _imuMes[topic].push_back(msg);

            } else if (RadarTopics.cend() != RadarTopics.find(topic)) {

                auto msg = radarDataLoaders.at(topic)->UnpackScan(item);
                _radarMes[topic].push_back(msg);

            }
        }
        bag->close();

        // if the radar is a AINSTEIN_RADAR radar, data should be reorganized,
        // i.e., merge multiple radar target measurements to radar array measurements
        std::map<std::string, std::vector<RadarTargetArray::Ptr>> oldRadarMes = _radarMes;
        for (const auto &[topic, loader]: radarDataLoaders) {
            if (loader->GetRadarModel() == RadarModelType::AWR1843BOOST_RAW ||
                loader->GetRadarModel() == RadarModelType::AWR1843BOOST_CUSTOM) {
                const auto &mes = oldRadarMes.at(topic);
                std::vector<RadarTarget::Ptr> targets;
                std::vector<RadarTargetArray::Ptr> arrays;
                for (const auto &item: mes) {
                    // merge measurements by 10 HZ (0.1 s)
                    if (targets.empty() || std::abs(targets.front()->GetTimestamp() - item->GetTimestamp()) < 0.1) {
                        targets.push_back(item->GetTargets().front());
                    } else {
                        // compute average time as the timestamp of radar target array
                        double t = 0.0;
                        for (const auto &target: targets) {
                            t += target->GetTimestamp() / static_cast<double>(targets.size());
                        }
                        arrays.push_back(RadarTargetArray::Create(t, targets));
                        targets.clear();
                        targets.push_back(item->GetTargets().front());
                    }
                }
                _radarMes.at(topic) = arrays;
            }
        }
        OutputDataStatus();
    }

    void CalibDataManager::AdjustCalibDataSequence() {
        spdlog::info("adjust calibration data sequence...");

        // make sure the first and last frame is imu frame
        _rawStartTimestamp =
                std::max_element(_imuMes.begin(), _imuMes.end(), [](const auto &p1, const auto &p2) {
                    return p1.second.front()->GetTimestamp() < p2.second.front()->GetTimestamp();
                })->second.front()->GetTimestamp();

        _rawEndTimestamp =
                std::min_element(_imuMes.begin(), _imuMes.end(), [](const auto &p1, const auto &p2) {
                    return p1.second.back()->GetTimestamp() < p2.second.back()->GetTimestamp();
                })->second.back()->GetTimestamp();

        double firstValidTimeStamp = _rawStartTimestamp + Configor::Prior::TimeOffsetPadding;
        double lastValidTimeStamp = _rawEndTimestamp - Configor::Prior::TimeOffsetPadding;

        // move lidar frames that are before the first imu frame and after the last imu frame
        for (const auto &[topic, _]: Configor::DataStream::RadarTopics) {
            // move radar frames that are before the first imu frame
            EraseSeqHeadData(_radarMes.at(topic), [firstValidTimeStamp](const RadarTargetArray::Ptr &frame) {
                return frame->GetTimestamp() > firstValidTimeStamp + Configor::Prior::TimeOffsetPadding;
            }, "the radar data is invalid, there is no intersection between imu data and radar data.");

            // move radar frames that are after the last imu frame
            EraseSeqTailData(_radarMes.at(topic), [lastValidTimeStamp](const RadarTargetArray::Ptr &frame) {
                return frame->GetTimestamp() < lastValidTimeStamp - Configor::Prior::TimeOffsetPadding;
            }, "the radar data is invalid, there is no intersection between imu data and radar data.");
        }

        OutputDataStatus();
    }

    void CalibDataManager::AlignTimestamp() {
        spdlog::info("align calibration data timestamp...");

        _alignedStartTimestamp = 0.0;
        _alignedEndTimestamp = _rawEndTimestamp - _rawStartTimestamp;
        for (auto &[imuTopic, mes]: _imuMes) {
            for (const auto &frame: mes) {
                frame->SetTimestamp(frame->GetTimestamp() - _rawStartTimestamp);
            }
        }
        for (const auto &[radarTopic, mes]: _radarMes) {
            for (const auto &array: mes) {
                // array
                array->SetTimestamp(array->GetTimestamp() - _rawStartTimestamp);
                // targets
                for (auto &tar: array->GetTargets()) {
                    tar->SetTimestamp(tar->GetTimestamp() - _rawStartTimestamp);
                }
            }
        }
        OutputDataStatus();
    }

    void CalibDataManager::OutputDataStatus() const {
        spdlog::info("calibration data info:");
        for (const auto &[imuTopic, mes]: _imuMes) {
            spdlog::info("IMU topic: '{}', data size: '{:06}', time span: from '{:+010.5f}' to '{:+010.5f}' (s)",
                         imuTopic, mes.size(), mes.front()->GetTimestamp(), mes.back()->GetTimestamp());
        }
        for (const auto &[radarTopic, mes]: _radarMes) {
            spdlog::info("Radar topic: '{}', data size: '{:06}', time span: from '{:+010.5f}' to '{:+010.5f}' (s)",
                         radarTopic, mes.size(), mes.front()->GetTimestamp(), mes.back()->GetTimestamp());
        }

        spdlog::info("raw start time: '{:+010.5f}' (s), raw end time: '{:+010.5f}' (s)",
                     _rawStartTimestamp, _rawEndTimestamp);
        spdlog::info("aligned start time: '{:+010.5f}' (s), aligned end time: '{:+010.5f}' (s)\n",
                     _alignedStartTimestamp, _alignedEndTimestamp);
    }

    double CalibDataManager::GetRawStartTimestamp() const {
        return _rawStartTimestamp;
    }

    double CalibDataManager::GetRawEndTimestamp() const {
        return _rawEndTimestamp;
    }

    double CalibDataManager::GetAlignedStartTimestamp() const {
        return _alignedStartTimestamp;
    }

    double CalibDataManager::GetAlignedEndTimestamp() const {
        return _alignedEndTimestamp;
    }

    const std::map<std::string, std::vector<IMUFrame::Ptr>> &CalibDataManager::GetIMUMeasurements() const {
        return _imuMes;
    }

    const std::vector<IMUFrame::Ptr> &CalibDataManager::GetIMUMeasurements(const std::string &imuTopic) const {
        return _imuMes.at(imuTopic);
    }

    const std::map<std::string, std::vector<RadarTargetArray::Ptr>> &CalibDataManager::GetRadarMeasurements() const {
        return _radarMes;
    }

    const std::vector<RadarTargetArray::Ptr> &
    CalibDataManager::GetRadarMeasurements(const std::string &radarTopic) const {
        return _radarMes.at(radarTopic);
    }

    bool CalibDataManager::SaveIMUMeasurements(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("imu_mes", this->_imuMes));
        return true;
    }

    bool CalibDataManager::SaveRadarMeasurements(const std::string &filename, CerealArchiveType::Enum archiveType) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        auto archive = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("radar_mes", this->_radarMes));
        return true;
    }
}