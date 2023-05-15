#pragma once

#include "spdlog/spdlog.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

#include <chrono>
#include <regex>

#include <filesystem>
#include <vector>

struct tmpstringstream {
    std::ostringstream ss;

    template<class T>
    tmpstringstream &operator<<(const T &val) {
        ss << val;
        return *this;
    }

    operator std::string() const { return ss.str(); }
};

struct ROSbag {
//    std::chrono::nanoseconds file_duration;
    std::string file_name;
    std::string file_path;
    std::string file_version;
    double file_size;
    std::map<std::string, std::vector<std::string>> topics_to_message_types;
    rosbag::Bag bag;

    ROSbag() {}

    void load_bag(const std::filesystem::path &path) {
        bag.open(path.string(), rosbag::bagmode::Read);

        rosbag::View entire_bag_view(bag);
        for (auto &&m: entire_bag_view) {
            topics_to_message_types[m.getTopic()].push_back(m.getDataType());
        }

        file_path = bag.getFileName();

        file_version = tmpstringstream() << bag.getMajorVersion() << "." << bag.getMinorVersion();
//        file_duration = get_duration(bag);
        file_size = 1.0 * bag.getSize() / (1024LL * 1024LL);
    }

//    std::chrono::nanoseconds get_duration(const rosbag::Bag &bag) { // Need to fix the regex expression
//        rosbag::View only_frames(bag, [](rosbag::ConnectionInfo const* info) {
//            std::regex exp(R"RRR(/device_\d+/sensor_\d+/.*_\d+/(image|imu))RRR");
//            return std::regex_search(info->topic, exp);
//        });
//        return std::chrono::nanoseconds((only_frames.getEndTime() - only_frames.getBeginTime()).toNSec());
//    }


};

//void read_dataset(const std::filesystem::path &path) {

//    spdlog::info("Reading dataset from {}", path.string());
//
//    ROSbag bag(path);
//
//    spdlog::info("File name: {}", bag.file_name);
//    spdlog::info("File path: {}", bag.file_path);
//    spdlog::info("File version: {}", bag.file_version);
//    spdlog::info("File size: {} MB", bag.file_size);
////    spdlog::info("File duration: {} s", bag.file_duration.count() / 1e9);
//    for (auto &&topic_to_message_type: bag.topics_to_message_types) {
//        spdlog::info("Topic: {}", topic_to_message_type.first);
//        for (auto &&message_type: topic_to_message_type.second) {
//            spdlog::info("Message type: {}", message_type);
//        }
//    }
//}