#include "recorder/dataset.hpp"
#include "spdlog/spdlog.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

namespace vk {
  void RosbagDatasetRecorder::callbackSyncedCameras(const std::vector<vk::CameraFrameData::Ptr> &dataVector) {
    if (!this->m_initialised) {
      spdlog::warn("Rosbag Dataset Recorder is not initialized yet!");
      return;
    }

    switch (this->m_mode) {
      case RecordMode::CONTINUOUS: {
        if (this->m_recording) {
          for (auto &frame: dataVector) {
            sensor_msgs::ImagePtr writeMsg;
            // Set the writeMsg params here from the
            writeMsg->height = frame->image.rows;
            writeMsg->width = frame->image.cols;
            writeMsg->encoding = frame->encoding;
            writeMsg->is_bigendian = false; // idk what this is
            writeMsg->step = frame->image.cols * frame->image.elemSize(); // full row length in bytes
            writeMsg->data.assign(frame->image.datastart, frame->image.dataend);

            std::string streamName = frame->prefixed_topic;

            this->m_bag->write(streamName, ros::Time(frame->ts), writeMsg);
          }
        }
      }
      case RecordMode::SNAPSHOTS: {
        if (this->m_recording && this->m_take_snapshot) {
          for (auto &frame: dataVector) {
            sensor_msgs::ImagePtr writeMsg;
            writeMsg->height = frame->image.rows;
            writeMsg->width = frame->image.cols;
            writeMsg->encoding = frame->encoding;
            writeMsg->is_bigendian = false; // idk what this is
            writeMsg->step = frame->image.cols * frame->image.elemSize(); // full row length in bytes
            writeMsg->data.assign(frame->image.datastart, frame->image.dataend); // copy

            std::string streamName = frame->prefixed_topic;

            this->m_bag->write(streamName, ros::Time(frame->ts), writeMsg);
          }
          this->m_num_snapshots++;
          this->m_take_snapshot = false;
        }
      }
      default:
        std::runtime_error("Undefined recording mode");
    }

  }

  void RosbagDatasetRecorder::callbackImu(const vk::ImuFrameData::Ptr data) {}

  void RosbagDatasetRecorder::start_record() {
    this->m_bag->open(this->m_bag_name, rosbag::bagmode::Write);
    this->m_recording = true;
  }

  void RosbagDatasetRecorder::stop_record() {
    this->m_recording = false;
    this->m_bag->close();
    spdlog::trace("Closing bag file at {}", this->m_bag_name);
  }
}