#include "recorder/dataset.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "spdlog/spdlog.h"

#include <cstring>

namespace vk {
    void RosbagDatasetRecorder::callbackSyncedCameras(const std::vector<vk::CameraFrameData::Ptr> &dataVector) {
        if (!this->m_initialised) {
            spdlog::warn("Rosbag Dataset Recorder is not initialized yet!");
            return;
        } else if (!this->m_recording) {
            return;
        }

        // Write to the this->display_imgs using the frame->image cv::Mat
        int idx = 0;
        for (auto &frame: dataVector) {
            this->m_display_imgs->at(idx) = frame->image;
            idx++;
        }

        std::lock_guard<std::mutex> lock(this->m_mutex);
        switch (this->m_mode) {
            case RecordMode::CONTINUOUS: {
                if (this->m_recording) {
                    for (auto &frame: dataVector) {
                        std::string streamName = frame->prefixed_topic;

                        sensor_msgs::ImagePtr writeMsg(new sensor_msgs::Image);

                        uint32_t sec = frame->ts / 1e9;
                        uint32_t nsec = frame->ts % uint32_t(1e9);
                        ros::Time timestamp(sec, nsec);
                        writeMsg->header.stamp = timestamp;
                        writeMsg->header.seq = frame->seq;

                        writeMsg->encoding = frame->encoding;

                        writeMsg->height = frame->image.rows;
                        writeMsg->width = frame->image.cols;

                        auto data_size = frame->image.total() * frame->image.elemSize();

                        writeMsg->data.resize(data_size);
                        std::memcpy(writeMsg->data.data(), frame->image.data, data_size);

                        this->m_bag->write(streamName, timestamp, writeMsg);
                    }
                    this->m_num_msgs++;
                }
                break;
            }
            case RecordMode::SNAPSHOTS: {
                if (this->m_recording && this->m_take_snapshot) {
                    for (auto &frame: dataVector) {
                        std::string streamName = frame->prefixed_topic;

                        sensor_msgs::ImagePtr writeMsg(new sensor_msgs::Image);

                        uint32_t sec = frame->ts / 1e9;
                        uint32_t nsec = frame->ts % uint32_t(1e9);
                        ros::Time timestamp(sec, nsec);
                        writeMsg->header.stamp = timestamp;
                        writeMsg->header.seq = frame->seq;

                        writeMsg->encoding = frame->encoding;

                        writeMsg->height = frame->image.rows;
                        writeMsg->width = frame->image.cols;

                        auto data_size = frame->image.total() * frame->image.elemSize();

                        writeMsg->data.resize(data_size);
                        std::memcpy(writeMsg->data.data(), frame->image.data, data_size);

                        this->m_bag->write(streamName, timestamp, writeMsg);
                    }
                    this->m_num_msgs++;
                    this->m_take_snapshot = false;
                }
                break;
            }
            default:
                throw std::runtime_error("Undefined recording mode");
                break;
        }
    }

    void RosbagDatasetRecorder::callbackImu(const vk::ImuFrameData::Ptr data) {}

    void RosbagDatasetRecorder::start_record() {
        // this->m_bag->open(this->m_bag_name, rosbag::bagmode::Write);
        this->m_recording = true;
    }

    void RosbagDatasetRecorder::stop_record() {
        this->m_recording = false;
        this->m_bag->close();
        spdlog::trace("Closing bag file at {}", this->m_bag_name);
    }
}// namespace vk
