#pragma once

#include "spdlog/spdlog.h"
#include <ecal/msg/capnproto/subscriber.h>
#include <ecal_camera/CameraFactory.hpp>
#include <rosbag/bag.h>
#include <utils/enum.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace vk {
    // NOLINTNEXTLINE
    BETTER_ENUM(RecordMode, int, SNAPSHOT = 0, CONTINUOUS = 1)

    class RosbagDatasetRecorder {
    public:
        typedef std::shared_ptr<RosbagDatasetRecorder> Ptr;

        void init(const vk::CameraParams &params, const RecordMode mode, std::shared_ptr<std::vector<cv::Mat>> &images) {
            this->m_mode = mode;
            this->m_recording = false;
            this->m_take_snapshot = false;
            this->m_display_imgs = images;
            this->m_camera = vk::CameraFactory::getCameraHandler();
            this->m_camera->init(params);// Setup GUI for default params

            this->m_camera->registerSyncedCameraCallback(std::bind(&RosbagDatasetRecorder::callbackSyncedCameras, this, std::placeholders::_1));

            // Set name for bag file and rosbag
            {
                std::ostringstream oss;
                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);

                oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%I-%M-%S");

                this->m_bag_name = oss.str() + this->m_mode._to_string() + "_recording.bag";

                this->m_bag = std::make_shared<rosbag::Bag>();
                this->m_bag->open(this->m_bag_name, rosbag::bagmode::Write);
            }

            this->m_initialised = true;
        }

        inline bool is_running() const { return this->m_recording; }
        inline RecordMode get_mode() const { return this->m_mode; }
        inline void take_snapshot() { this->m_take_snapshot = true; }
        inline int get_num_snapshots() { return this->m_num_msgs; }

        void start_record();
        void stop_record();

    private:
        bool m_initialised;// Set to true after callback is passed
        RecordMode m_mode = RecordMode::SNAPSHOT;
        vk::CameraInterface::Ptr m_camera;

        std::string m_bag_name;
        std::shared_ptr<rosbag::Bag> m_bag;
        std::mutex m_mutex;// Safe access of m_bag
                           //    std::shared_ptr<std::thread> m_thread_image;
                           //    std::shared_ptr<std::thread> m_thread_imu;
        std::atomic<int> m_num_msgs;
        std::atomic<bool> m_take_snapshot;
        std::atomic<bool> m_recording;

        std::shared_ptr<std::vector<cv::Mat>> m_display_imgs;

        void callbackSyncedCameras(const std::vector<vk::CameraFrameData::Ptr> &dataVector);
        void callbackImu(vk::ImuFrameData::Ptr data);
    };
}// namespace vk