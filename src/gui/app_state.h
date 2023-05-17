#pragma once

#include "io/dataset_io.h"
#include "io/dataset_io_rosbag.h"
#include "calibration/aprilgrid.h"
#include "rosbag_inspector.h"
#include "calibration/aprilgrid.h"
#include "calibration/calibration_helper.h"
#include <basalt/serialization/headers_serialization.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>

using namespace basalt;

struct AppState1 {
  // All other configs, can look at the field vars of VioDataset or RosbagIO to check, keep this struct empty
  std::string file_path; // Only able to store one .bag file at a time
  double file_size;
  RosbagIO rosbag_io;
//  std::map<std::string, std::vector<std::string>> topics_to_message_types;
  //basalt::VioDatasetPtr &dataset; // Can access the rosbag file by pressing .bag here
  std::vector<int64_t> image_timestamps;
  // map from timestamp to cv::Mat
  std::map<int64_t, std::vector<cv::Mat>> image_data; // Must be ready to plot
  AprilGrid april_grid;
  CalibCornerMap calib_corners;
  CalibCornerMap calib_corners_rejected;

  AppState1(std::string april_grid_path) : april_grid(april_grid_path) {};

  void loadDataset(const std::filesystem::path &path) {
    this->file_path = path.string();
    this->rosbag_io.read(path);

    //this->dataset = this->rosbag_io.get_data(); // data is a private member of rosbag_io, so copy reference
    // Replace dataset with rosbag_io.get_data() because shared_ptr

    this->file_size = this->rosbag_io.get_data()->get_size();

    this->image_timestamps = this->rosbag_io.get_data()->get_image_timestamps();
    spdlog::info("Loaded {} timestamps into memory", this->image_timestamps.size());

    this->feed_images();

    // Load detected corners if cached
    {
      std::string path = "/Users/tejas/Developer/vilota-dev/calibration_tool/data/stuff_detected_corners.cereal";

      std::ifstream is(path, std::ios::binary);

      if (is.good()) {
        cereal::BinaryInputArchive archive(is);

        calib_corners.clear();
        calib_corners_rejected.clear();
        archive(calib_corners);
        archive(calib_corners_rejected);

        std::cout << "Loaded detected corners from: " << path << std::endl;
      } else {
        std::cout << "No pre-processed detected corners found" << std::endl;
      }
    }
  }

  void feed_images() {
    for (int64_t t_ns: this->image_timestamps) {
      std::vector<cv::Mat> cv_img_vec; // One vector for each timestamp
      const std::vector<ImageData> &img_vec = this->rosbag_io.get_data()->get_image_data(t_ns);

      for (size_t cam_id = 0; cam_id < this->rosbag_io.get_data()->get_num_cams(); cam_id++) {
        const ManagedImage<uint16_t>::Ptr &managed_image_ptr = img_vec[cam_id].img;

        // Check if the managed_image_ptr is valid
        if (managed_image_ptr) {
          // Extract the image properties from the managed_image_ptr
          size_t width = managed_image_ptr->w;
          size_t height = managed_image_ptr->h;
          size_t pitch_bytes = managed_image_ptr->pitch;

          // Create a cv::Mat with the appropriate size and data type
          cv::Mat image_mat_16u(height, width, CV_16U, managed_image_ptr->ptr, pitch_bytes);

          cv::Mat image_mat_8u;
          image_mat_16u.convertTo(image_mat_8u, CV_8U, 1.0 / 256.0);

          cv_img_vec.push_back(image_mat_8u);

          // Do something with the converted cv::Mat (e.g., perform image processing or display)
          // ...

          // Note: The cv::Mat `image_mat_8u` contains the scaled-down 8-bit values.
          // Make sure the ManagedImage remains valid during the lifetime of `image_mat_8u`.
          // otherwise, the cv::Mat will contain invalid data.
        }
      }

      this->image_data.insert({t_ns, cv_img_vec});
    }

    spdlog::info("Loaded {} timestamps into memory", this->image_timestamps.size());
  }


  // Convert from ManagedImage to cv::Mat
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

  void drawImageOverlay(cv::Mat image) {
    int64_t timestamp_ns = rosbag_io.get_data()->get_image_timestamps()[0];
    TimeCamId tcid(timestamp_ns, 0);

    const CalibCornerData &cr = calib_corners.at(tcid);
    const CalibCornerData &cr_rej = calib_corners_rejected.at(tcid);

    for (size_t i = 0; i < cr.corners.size(); i++) {
      // The radius is the threshold used for maximum displacement.
      // The search region is slightly larger.
      const float radius = static_cast<float>(cr.radii[i]);
      const Eigen::Vector2d& c = cr.corners[i];

      cv::circle(image, cv::Point2d(c[0], c[1]), static_cast<int>(radius),
                 cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    }
  }
};