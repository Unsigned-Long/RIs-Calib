// Copyright (c) 2023. Created on 9/20/23 1:17 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.


#include <utility>
#include "rosbag/view.h"
#include "rosbag/bag.h"
#include "ros/ros.h"
#include "util/utils.hpp"
#include "calib/status.hpp"
#include "spdlog/spdlog.h"

namespace ns_ris {

    struct MergeConfigor {
    public:
        using Ptr = std::shared_ptr<MergeConfigor>;

        struct BagMergeInfo {
            std::string bagPath;
            std::map<std::string, std::string> topicsToMerge;

            [[nodiscard]] std::string GetDstTopic(const std::string &srcTopic) const {
                auto iter = topicsToMerge.find(srcTopic);
                return (iter == topicsToMerge.end() ? srcTopic : iter->second);
            }

            [[nodiscard]] std::vector<std::string> GetSrcTopicVec() const {
                std::vector<std::string> topics;
                std::transform(
                        topicsToMerge.begin(), topicsToMerge.end(), std::back_inserter(topics),
                        [](const auto &p) { return p.first; }
                );
                return topics;
            }

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(bagPath), CEREAL_NVP(topicsToMerge));
            }
        };

    public:
        std::vector<BagMergeInfo> _bags;
        std::string _outputBagPath;

        MergeConfigor() = default;

        static Ptr Create() {
            return std::make_shared<MergeConfigor>();
        }

        // load configure information from file
        static Ptr LoadConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                return nullptr;
            }
            auto archive = GetInputArchiveVariant(file, archiveType);
            auto configor = MergeConfigor::Create();
            SerializeByInputArchiveVariant(archive, archiveType, cereal::make_nvp("MergeConfigor", *configor));
            return configor;
        }

        // save configure information to file
        bool SaveConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }
            auto archive = GetOutputArchiveVariant(file, archiveType);
            SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("MergeConfigor", *this));
            return true;
        }

        void PrintMainFields() {
            for (const auto &bag: _bags) {
                std::stringstream stream;
                for (const auto &[srcTopic, dstTopic]: bag.topicsToMerge) {
                    stream << "{'" << srcTopic << "' -> '" << dstTopic << "'}";
                }
                spdlog::info("rosbag path: '{}', topics to merge in this bag: {}.", bag.bagPath, stream.str());
            }
            spdlog::info("output rosbag path: '{}'.", _outputBagPath);
        }

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("Bags", _bags), cereal::make_nvp("OutputBagPath", _outputBagPath));
        }
    };

    class BagMerger {
    public:
        using Ptr = std::shared_ptr<BagMerger>;

    private:
        MergeConfigor::Ptr _configor;
    public:
        explicit BagMerger(MergeConfigor::Ptr mergeConfigor) : _configor(std::move(mergeConfigor)) {}

        static Ptr Create(const MergeConfigor::Ptr &mergeConfigor) {
            return std::make_shared<BagMerger>(mergeConfigor);
        }

        void Process() {
            spdlog::info("start merge {} bag(s) to '{}'.", _configor->_bags.size(), _configor->_outputBagPath);
            auto dstBag = std::make_unique<rosbag::Bag>();
            dstBag->open(_configor->_outputBagPath, rosbag::BagMode::Write);
            for (const auto &bagInfo: _configor->_bags) {
                spdlog::info("process bag at '{}'...", bagInfo.bagPath);
                // open current rosbag
                auto srcBag = std::make_unique<rosbag::Bag>();
                srcBag->open(bagInfo.bagPath, rosbag::BagMode::Read);
                // query
                auto view = rosbag::View();
                view.addQuery(*srcBag, rosbag::TopicQuery(bagInfo.GetSrcTopicVec()));
                for (const auto &item: view) {
                    dstBag->write(
                            bagInfo.GetDstTopic(item.getTopic()), item.getTime(), item, item.getConnectionHeader()
                    );
                }
                srcBag->close();
            }
            dstBag->close();
            spdlog::info("process finished...");
        }
    };
}

