// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_CALIB_DATA_MANAGER_H
#define RIS_CALIB_CALIB_DATA_MANAGER_H

#include "config/configor.h"
#include "sensor/imu.h"
#include "sensor/radar.h"
#include "calib/status.hpp"

namespace ns_ris {

    class CalibDataManager {
    public:
        using Ptr = std::shared_ptr<CalibDataManager>;

    private:
        std::map<std::string, std::vector<IMUFrame::Ptr>> _imuMes;
        std::map<std::string, std::vector<RadarTargetArray::Ptr>> _radarMes;

        double _rawStartTimestamp{};
        double _rawEndTimestamp{};

        double _alignedStartTimestamp{};
        double _alignedEndTimestamp{};

    public:
        // using config information to load and adjust data in this constructor
        CalibDataManager();

        // the creator
        static CalibDataManager::Ptr Create();

        // get raw imu frames
        [[nodiscard]] const std::map<std::string, std::vector<IMUFrame::Ptr>> &
        GetIMUMeasurements() const;

        [[nodiscard]] const std::vector<IMUFrame::Ptr> &
        GetIMUMeasurements(const std::string &imuTopic) const;

        // get raw radar frames
        [[nodiscard]] const std::map<std::string, std::vector<RadarTargetArray::Ptr>> &
        GetRadarMeasurements() const;

        [[nodiscard]] const std::vector<RadarTargetArray::Ptr> &
        GetRadarMeasurements(const std::string &radarTopic) const;

        [[nodiscard]] double GetRawStartTimestamp() const;

        [[nodiscard]] double GetRawEndTimestamp() const;

        [[nodiscard]] double GetAlignedStartTimestamp() const;

        [[nodiscard]] double GetAlignedEndTimestamp() const;

        bool SaveIMUMeasurements(const std::string &filename, CerealArchiveType::Enum archiveType);

        bool SaveRadarMeasurements(const std::string &filename, CerealArchiveType::Enum archiveType);

    protected:
        // load camera, lidar, imu data from the ros bag [according to the config file]
        void LoadCalibData();

        // make sure the first imu frame is before camera and lidar data
        // assign the '_alignedStartTimestamp' and '_alignedEndTimestamp'
        void AdjustCalibDataSequence();

        // align the timestamp to zero
        void AlignTimestamp();

        // remove the head data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqHeadData(std::vector<ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.begin(), seq.end(), pred);
            if (iter == seq.end()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(seq.begin(), iter);
            }
        }

        // remove the tail data according to the pred
        template<typename ElemType, typename Pred>
        void EraseSeqTailData(std::vector<ElemType> &seq, Pred pred, const std::string &errorMsg) {
            auto iter = std::find_if(seq.rbegin(), seq.rend(), pred);
            if (iter == seq.rend()) {
                // find failed
                throw Status(Status::Flag::ERROR, errorMsg);
            } else {
                // adjust
                seq.erase(iter.base(), seq.end());
            }
        }

        // output the data status
        void OutputDataStatus() const;
    };

}


#endif //RIS_CALIB_CALIB_DATA_MANAGER_H
