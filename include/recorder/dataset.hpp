#pragma once

#include "presets.hpp"
#include "better_bag.hpp"

#include "spdlog/spdlog.h"
#include <ecal/msg/capnproto/subscriber.h>
#include <ecal_camera/CameraFactory.hpp>
#define private public
#include <rosbag/bag.h>
#include <rosbag/view.h>
#undef private
#include <utils/enum.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <numeric>

namespace vk {
    // NOLINTNEXTLINE
    BETTER_ENUM(RecordMode, int, SNAPSHOT = 0, CONTINUOUS = 1)

    class RosbagDatasetRecorder {
    public:
        typedef std::shared_ptr<RosbagDatasetRecorder> Ptr;

        void init(const Preset& preset, const RecordMode mode, std::shared_ptr<std::unordered_map<std::string, cv::Mat>> &images) {
            this->m_mode = mode;
            this->m_recording = false;
            this->m_take_snapshot = false;
            this->m_display_imgs = images;
            // Resize the images vector to be params->size()
            size_t num_cameras = std::accumulate(preset.get_params().begin(), preset.get_params().end(), 0,
                                                 [](size_t sum, const auto& param) { return sum + param.camera_topics.size(); });
            // this->m_display_imgs->resize(num_cameras);

            // For each of the vk::CameraParams in the preset, create a new CameraInterface
            // Not sure if std::move-ing after initializing and registering callback will cause problems, so
            // will just create a new CameraInterface and move it into the vector
            for (int i = 0; i < preset.get_params().size(); i++) {
                auto camera = vk::CameraFactory::getCameraHandler();
                this->m_cameras.push_back(std::move(camera));
                this->m_cameras[i]->init(preset.get_params()[i]);
                this->m_cameras[i]->registerSyncedCameraCallback(std::bind(&RosbagDatasetRecorder::callbackSyncedCameras, this, std::placeholders::_1));
            }

            // Set name for bag file and rosbag
            {
                std::ostringstream oss;
                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);

                oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%I-%M-%S");

                this->m_bag_name = oss.str() + this->m_mode._to_string() + "_recording.bag";

                this->m_better_bag = std::make_shared<BetterBag>();
                this->m_better_bag->openWrite(this->m_bag_name);
            }

            this->m_initialised = true;
        }

        [[nodiscard]] inline bool is_init() const { return this->m_initialised; }
        [[nodiscard]] inline bool is_running() const { return this->m_recording; }
        [[nodiscard]] inline RecordMode get_mode() const { return this->m_mode; }
        inline void take_snapshot() {
            // for each of the unique prefixes, set the corresponding bool in m_take_snapshot_map to be true
            for (const auto& camera : this->m_cameras) {
                this->m_take_snapshot_map[camera->getParams().tf_prefix] = true;
            }
        }
        inline int get_num_snapshots() { return this->m_num_msgs; }

        void start_record();
        void stop_record();

    private:
        bool m_initialised;// Set to true after callback is passed
        RecordMode m_mode = RecordMode::SNAPSHOT;
        std::vector<CameraInterface::Ptr> m_cameras;

        std::string m_bag_name;
        std::shared_ptr<BetterBag> m_better_bag;
        std::atomic<int> m_num_msgs;
        std::atomic<bool> m_take_snapshot;
        std::unordered_map<std::string, std::atomic<bool>> m_take_snapshot_map; // e.g "S0" -> true if need to take a shot
        std::atomic<bool> m_recording;

        std::shared_ptr<std::unordered_map<std::string, cv::Mat>> m_display_imgs;

        void callbackSyncedCameras(const std::vector<vk::CameraFrameData::Ptr> &dataVector);
        void callbackImu(vk::ImuFrameData::Ptr data);
    };
}// namespace vk