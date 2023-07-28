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

        for (auto &frame: dataVector) {
            (*this->m_display_imgs)[frame->prefixed_topic] = frame->image;
        }

        std::string prefix = dataVector[0]->prefixed_topic.substr(0, 3);

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
                        // if mono thens step size is width.

                        writeMsg->height = frame->image.rows;
                        writeMsg->width = frame->image.cols;
//                        writeMsg->step = frame->w

                        auto data_size = frame->image.total() * frame->image.elemSize();

                        writeMsg->data.resize(data_size);
                        std::memcpy(writeMsg->data.data(), frame->image.data, data_size);
                        this->m_better_bag->write(streamName, timestamp, writeMsg);
                    }
                    this->m_num_msgs++;
                }
                break;
            }
            case RecordMode::SNAPSHOT: {
                if (this->m_recording && this->m_take_snapshot_map[prefix]) {
                    for (auto &frame: dataVector) {
                        std::string streamName = frame->prefixed_topic;
                        spdlog::debug("I AM INSIDE: {}", streamName);

                        sensor_msgs::ImagePtr writeMsg(new sensor_msgs::Image);

                        uint32_t sec = frame->ts / 1e9;
                        uint32_t nsec = frame->ts % uint32_t(1e9);
                        ros::Time timestamp(sec, nsec);
                        writeMsg->header.stamp = timestamp;
                        writeMsg->header.seq = frame->seq;

                        writeMsg->encoding = frame->encoding;
                        spdlog::trace("Encoding: {}", frame->encoding);

                        writeMsg->height = frame->image.rows;
                        writeMsg->width = frame->image.cols;

                        auto data_size = frame->image.total() * frame->image.elemSize();

                        writeMsg->data.resize(data_size);
                        std::memcpy(writeMsg->data.data(), frame->image.data, data_size);

                        this->m_better_bag->write(streamName, timestamp, writeMsg);
                    }
                    this->m_num_msgs++;
                    this->m_take_snapshot_map[prefix] = false;
                }
                break;
            }
            default:
                throw std::runtime_error("Undefined recording mode");
        }
    }

    void RosbagDatasetRecorder::callbackImu(const vk::ImuFrameData::Ptr data) {}

    void RosbagDatasetRecorder::start_record() {
        // Already open bag in write mode in the init
        this->m_recording = true;
    }

    void RosbagDatasetRecorder::stop_record() {
        this->m_recording = false; // for callbacks to stop recording as well
        this->m_better_bag->close();
    }
}// namespace vk
