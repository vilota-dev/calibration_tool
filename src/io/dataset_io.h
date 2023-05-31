#pragma once

#include "utils/filesystem.h"
#include "calibration/calibration_data.h"

#include <basalt/camera/generic_camera.hpp>
#include <basalt/camera/stereographic_param.hpp>
#include <basalt/image/image.h>
#include <basalt/utils/assert.h>
#include <basalt/utils/sophus_utils.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/bitset.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>
// Hack to access private functions
#define private public

#include <rosbag/bag.h>
#include <rosbag/view.h>

#undef private

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "spdlog/spdlog.h"

#include <array>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <optional>
#include <functional>

namespace basalt {
  cv::Mat convert(ManagedImage<uint16_t>::Ptr img) {
    // Create a cv::Mat with the appropriate size and data type
    cv::Mat image_mat_16u(img->h, img->w, CV_16U, img->ptr, img->pitch);

    // Convert the 16-bit image to 8-bit
    cv::Mat img_8u;
    image_mat_16u.convertTo(img_8u, CV_8U, 1.0 / 256.0);

    // Create a 3-channel color image
    cv::Mat img_color(img_8u.size(), CV_8UC3);

    // Copy the grayscale image to all three color channels
    cv::cvtColor(img_8u, img_color, cv::COLOR_GRAY2BGR);

    return img_color;
  }

  struct ImageData {
    ImageData() : exposure(0) {}

    ManagedImage<uint16_t>::Ptr img;
    double exposure;
  };

  struct Observations {
    Eigen::aligned_vector<Eigen::Vector2d> pos;
    std::vector<int> id;
  };

  struct GyroData {
    int64_t timestamp_ns;
    Eigen::Vector3d data;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct AccelData {
    int64_t timestamp_ns;
    Eigen::Vector3d data;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct PoseData {
    int64_t timestamp_ns;
    Sophus::SE3d data;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct MocapPoseData {
    int64_t timestamp_ns;
    Sophus::SE3d data;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct AprilgridCornersData {
    int64_t timestamp_ns;
    int cam_id;

    Eigen::aligned_vector<Eigen::Vector2d> corner_pos;
    std::vector<int> corner_id;
  };

  class RosbagDataset {
    std::string file_path;
    double file_size;
    std::shared_ptr<rosbag::Bag> bag;
    std::mutex m;

    size_t num_cams;

    std::vector<int64_t> image_timestamps;

    // vector of images for every timestamp
    // assumes vectors size is num_cams for every timestamp with null pointers for
    // missing frames
    std::unordered_map<int64_t, std::vector<std::optional<rosbag::IndexEntry>>>
            image_data_idx;

    Eigen::aligned_vector<AccelData> accel_data;
    Eigen::aligned_vector<GyroData> gyro_data;

    std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
    Eigen::aligned_vector<Sophus::SE3d>
            gt_pose_data;  // TODO: change to eigen aligned

    int64_t mocap_to_imu_offset_ns;

    std::set<std::string> cam_topics;
    std::string imu_topic;

  public:

    RosbagDataset(const std::string &path) {
      spdlog::debug("Creating rosbag dataset");
      read(path);
    }

    ~RosbagDataset() {}

    double get_file_size() { return this->file_size; }

    std::shared_ptr<rosbag::Bag> get_bag() {
      return this->bag;
    }

    std::string get_file_path() const {
      return this->file_path;
    }

    size_t get_num_cams() const { return num_cams; }

    std::set<std::string> get_camera_names() { return cam_topics; }

    std::string get_imu_name() { return imu_topic; }

    std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }

    const Eigen::aligned_vector<AccelData> &get_accel_data() const {
      return accel_data;
    }

    const Eigen::aligned_vector<GyroData> &get_gyro_data() const {
      return gyro_data;
    }

    const std::vector<int64_t> &get_gt_timestamps() const {
      return gt_timestamps;
    }

    const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
      return gt_pose_data;
    }

    int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

    // Return the topic to message type map
    std::map<std::string, std::vector<std::string>> topics_to_message_types;

    // Store as public member, the move semantics gets confusing and alot of copies made
    // Use a mutex to protect access, once there are more threads, but for now there's only one so it
    // doesn't matter.
    CalibCornerMap calib_corners;
    CalibCornerMap calib_corners_rejected;

    std::map<int64_t, std::vector<cv::Mat>> image_data; // corners will be drawn into memory for now

    void read(const std::string &path) {
      if (!fs::exists(path)) {
        spdlog::error("No dataset found in {}", path);
      } else {
        spdlog::trace("Reading rosbag dataset from {}", path);
      }

      this->file_path = path;
//      this->file_size = 1.0 * bag->getSize() / (1024LL * 1024LL); // always causes segfault
//      this->file_size = 1.0 * fs::file_size(path) / (1024LL * 1024LL);

      this->bag = std::make_shared<rosbag::Bag>();
      this->bag->open(path, rosbag::bagmode::Read);

      rosbag::View view(*this->bag);

      for (auto &&m: view) {
        topics_to_message_types[m.getTopic()].push_back(m.getDataType());
      }

      // get topics
      std::vector<const rosbag::ConnectionInfo *> connection_infos =
              view.getConnections();

      auto &cam_topics = this->cam_topics;
      auto &imu_topic = this->imu_topic;
      std::string mocap_topic;
      std::string point_topic;

      for (const rosbag::ConnectionInfo *info: connection_infos) {
        //      if (info->topic.substr(0, 4) == std::string("/cam")) {
        //        cam_topics.insert(info->topic);
        //      } else if (info->topic.substr(0, 4) == std::string("/imu")) {
        //        imu_topic = info->topic;
        //      } else if (info->topic.substr(0, 5) == std::string("/vrpn") ||
        //                 info->topic.substr(0, 6) == std::string("/vicon")) {
        //        mocap_topic = info->topic;
        //      }

        if (info->datatype == std::string("sensor_msgs/Image")) {
          cam_topics.insert(info->topic);
        } else if (info->datatype == std::string("sensor_msgs/Imu") &&
                   info->topic.rfind("/fcu", 0) != 0) {
          imu_topic = info->topic;
        } else if (info->datatype ==
                   std::string("geometry_msgs/TransformStamped") ||
                   info->datatype == std::string("geometry_msgs/PoseStamped")) {
          mocap_topic = info->topic;
        } else if (info->datatype == std::string("geometry_msgs/PointStamped")) {
          point_topic = info->topic;
        }
      }

      spdlog::debug("imu_topic: {}", imu_topic);
      spdlog::debug("mocap_topic: {}", mocap_topic);
      std::string topics_str;
      for (const std::string &s: cam_topics)
        topics_str += s + " ";
      spdlog::debug("cam_topics: {}", topics_str);


      std::map<std::string, int> topic_to_id;
      int idx = 0;
      for (const std::string &s: cam_topics) {
        topic_to_id[s] = idx;
        idx++;
      }

      this->num_cams = cam_topics.size();

      int num_msgs = 0;

      int64_t min_time = std::numeric_limits<int64_t>::max();
      int64_t max_time = std::numeric_limits<int64_t>::min();

      std::vector<geometry_msgs::TransformStampedConstPtr> mocap_msgs;
      std::vector<geometry_msgs::PointStampedConstPtr> point_msgs;

      std::vector<int64_t>
              system_to_imu_offset_vec;  // t_imu = t_system + system_to_imu_offset
      std::vector<int64_t> system_to_mocap_offset_vec;  // t_mocap = t_system +
      // system_to_mocap_offset

      std::set < int64_t > image_timestamps;

      for (const rosbag::MessageInstance &m: view) {
        const std::string &topic = m.getTopic();

        if (cam_topics.find(topic) != cam_topics.end()) {
          sensor_msgs::ImageConstPtr img_msg =
                  m.instantiate<sensor_msgs::Image>();
          int64_t timestamp_ns = img_msg->header.stamp.toNSec();

          auto &img_vec = this->image_data_idx[timestamp_ns];
          if (img_vec.size() == 0) img_vec.resize(this->num_cams);

          img_vec[topic_to_id.at(topic)] = m.index_entry_;
          image_timestamps.insert(timestamp_ns);

          min_time = std::min(min_time, timestamp_ns);
          max_time = std::max(max_time, timestamp_ns);
        }

        if (imu_topic == topic) {
          sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
          int64_t time = imu_msg->header.stamp.toNSec();

          this->accel_data.emplace_back();
          this->accel_data.back().timestamp_ns = time;
          this->accel_data.back().data = Eigen::Vector3d(
                  imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                  imu_msg->linear_acceleration.z);

          this->gyro_data.emplace_back();
          this->gyro_data.back().timestamp_ns = time;
          this->gyro_data.back().data = Eigen::Vector3d(
                  imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                  imu_msg->angular_velocity.z);

          min_time = std::min(min_time, time);
          max_time = std::max(max_time, time);

          int64_t msg_arrival_time = m.getTime().toNSec();
          system_to_imu_offset_vec.push_back(time - msg_arrival_time);
        }

        if (mocap_topic == topic) {
          geometry_msgs::TransformStampedConstPtr mocap_msg =
                  m.instantiate<geometry_msgs::TransformStamped>();

          // Try different message type if instantiate did not work
          if (!mocap_msg) {
            geometry_msgs::PoseStampedConstPtr mocap_pose_msg =
                    m.instantiate<geometry_msgs::PoseStamped>();

            geometry_msgs::TransformStampedPtr mocap_new_msg(
                    new geometry_msgs::TransformStamped);
            mocap_new_msg->header = mocap_pose_msg->header;
            mocap_new_msg->transform.rotation = mocap_pose_msg->pose.orientation;
            mocap_new_msg->transform.translation.x =
                    mocap_pose_msg->pose.position.x;
            mocap_new_msg->transform.translation.y =
                    mocap_pose_msg->pose.position.y;
            mocap_new_msg->transform.translation.z =
                    mocap_pose_msg->pose.position.z;

            mocap_msg = mocap_new_msg;
          }

          int64_t time = mocap_msg->header.stamp.toNSec();

          mocap_msgs.push_back(mocap_msg);

          int64_t msg_arrival_time = m.getTime().toNSec();
          system_to_mocap_offset_vec.push_back(time - msg_arrival_time);
        }

        if (point_topic == topic) {
          geometry_msgs::PointStampedConstPtr mocap_msg =
                  m.instantiate<geometry_msgs::PointStamped>();

          int64_t time = mocap_msg->header.stamp.toNSec();

          point_msgs.push_back(mocap_msg);

          int64_t msg_arrival_time = m.getTime().toNSec();
          system_to_mocap_offset_vec.push_back(time - msg_arrival_time);
        }

        num_msgs++;
      }

      this->image_timestamps.clear();
      this->image_timestamps.insert(this->image_timestamps.begin(),
                                    image_timestamps.begin(),
                                    image_timestamps.end());

      if (system_to_mocap_offset_vec.size() > 0) {
        int64_t system_to_imu_offset =
                system_to_imu_offset_vec[system_to_imu_offset_vec.size() / 2];

        int64_t system_to_mocap_offset =
                system_to_mocap_offset_vec[system_to_mocap_offset_vec.size() / 2];

        this->mocap_to_imu_offset_ns =
                system_to_imu_offset - system_to_mocap_offset;
      }

      this->gt_pose_data.clear();
      this->gt_timestamps.clear();

      if (!mocap_msgs.empty())
        for (size_t i = 0; i < mocap_msgs.size() - 1; i++) {
          auto mocap_msg = mocap_msgs[i];

          int64_t time = mocap_msg->header.stamp.toNSec();

          Eigen::Quaterniond q(
                  mocap_msg->transform.rotation.w, mocap_msg->transform.rotation.x,
                  mocap_msg->transform.rotation.y, mocap_msg->transform.rotation.z);

          Eigen::Vector3d t(mocap_msg->transform.translation.x,
                            mocap_msg->transform.translation.y,
                            mocap_msg->transform.translation.z);

          int64_t timestamp_ns = time + this->mocap_to_imu_offset_ns;
          this->gt_timestamps.emplace_back(timestamp_ns);
          this->gt_pose_data.emplace_back(q, t);
        }

      if (!point_msgs.empty())
        for (size_t i = 0; i < point_msgs.size() - 1; i++) {
          auto point_msg = point_msgs[i];

          int64_t time = point_msg->header.stamp.toNSec();

          Eigen::Vector3d t(point_msg->point.x, point_msg->point.y,
                            point_msg->point.z);

          int64_t timestamp_ns = time;  // + data->mocap_to_imu_offset_ns;
          this->gt_timestamps.emplace_back(timestamp_ns);
          this->gt_pose_data.emplace_back(Sophus::SO3d(), t);
        }

      spdlog::debug("Total number of messages: {}", num_msgs);
      spdlog::debug("Image size: {}", this->image_data_idx.size());
      spdlog::debug("Min time: {} | Max time: {} | mocap to imu offset: {}",
                    min_time, max_time, this->mocap_to_imu_offset_ns);
      spdlog::debug("Number of mocap poses: {}", this->gt_timestamps.size());

      // Section: Read the images and convert them to cv::Mat and store them in image_data
      for (auto ts : this->image_timestamps) {
        std::vector<ImageData> raw_data = this->get_image_data(ts);

        if (raw_data.empty()) {
          spdlog::error("No image data found for timestamp {}", ts);
          continue;
        }
        // std::map<int64_t, std::vector<cv::Mat>> image_data;
        std::vector<cv::Mat> converted_images;

        // Convert the images to cv::Mat
        for (auto & i : raw_data) {
          auto temp = convert(i.img);
          converted_images.push_back(temp);
        }

        // Put the converted images in the map
        this->image_data[ts] = converted_images;
      }

      spdlog::debug("Successfully converted into {} cv::Mat images", this->image_data.size());

    }

    std::vector<ImageData> get_image_data(int64_t t_ns) {
      spdlog::debug("RosbagDataset::get_image_data");
      std::vector<ImageData> res(num_cams);

      auto it = image_data_idx.find(t_ns);

      if (it != image_data_idx.end())
        for (size_t i = 0; i < num_cams; i++) {
          ImageData &id = res[i];

          if (!it->second[i].has_value()) continue;

          m.lock();
          sensor_msgs::ImageConstPtr img_msg =
                  bag->instantiateBuffer<sensor_msgs::Image>(*it->second[i]);
          m.unlock();

          //        std::cerr << "img_msg->width " << img_msg->width << "
          //        img_msg->height "
          //                  << img_msg->height << std::endl;

          id.img.reset(
                  new ManagedImage<uint16_t>(img_msg->width, img_msg->height));

          if (!img_msg->header.frame_id.empty() &&
              std::isdigit(img_msg->header.frame_id[0])) {
            id.exposure = std::stol(img_msg->header.frame_id) * 1e-9;
          } else {
            id.exposure = -1;
          }

          if (img_msg->encoding == "mono8") {
            const uint8_t *data_in = img_msg->data.data();
            uint16_t *data_out = id.img->ptr;

            for (size_t i = 0; i < img_msg->data.size(); i++) {
              int val = data_in[i];
              val = val << 8;
              data_out[i] = val;
            }

          } else if (img_msg->encoding == "mono16") {
            std::memcpy(id.img->ptr, img_msg->data.data(), img_msg->data.size());
          } else if (img_msg->encoding == "rgb8") {
            // take only the first channel
            const uint8_t *data_in = img_msg->data.data();
            uint16_t *data_out = id.img->ptr;

            for (size_t i = 0; i < img_msg->data.size(); i += 3) {
              int val = data_in[i];
              val = val << 8;
              data_out[i / 3] = val;
            }
          } else {
            std::cerr << "Encoding " << img_msg->encoding << " is not supported."
                      << std::endl;
            std::abort();
          }
        }

      return res;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend class RosbagIO;
  };

  typedef std::shared_ptr<RosbagDataset> RosbagDatasetPtr;
}  // namespace basalt


