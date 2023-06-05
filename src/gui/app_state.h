#pragma once

#include "calibration/calibration_helper.h"
#include "io/aprilgrid_container.h"
#include "io/rosbag_container.h"

#include "imports.h"
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/plot_corners.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

using namespace basalt;

struct AppState {
  RosbagContainer rosbag_files;
  int selected; // selected rosbag_file for display
  AprilGridContainer aprilgrid_files;
  cbdetect::Params checkerboard_params;

  std::map<std::string, uint64_t> num_topics_to_show; // for the rosbag inspector config

  int selectedFrame;

  std::shared_ptr<std::thread> processing_thread;

  ImmVision::ImageParams immvisionParams;

  // Delete later, just for show
  std::vector<float> frame_rates; // Doesn't need to be shared_ptr, since the app_state object will remain alive till end of the main function

  AppState() {
    spdlog::trace("Initializing AppState object");
    this->selected = 0; // for the selected rosbag file
    this->selectedFrame = 0;
    immvisionParams = ImmVision::ImageParams();
    immvisionParams.ImageDisplaySize = cv::Size(600, 0);
    immvisionParams.ZoomKey = "z";
    immvisionParams.RefreshImage = true;

    // Checkerboard params config
    this->checkerboard_params.show_processing = true; // Prints the shit out.

    // Delete later
    for (int i = 0; i < 200; ++i) {
      frame_rates.push_back(0.f);
    }
  };

  ~AppState() {
    spdlog::trace("Destroying AppState object and joining processing thread");
    if (processing_thread) {
      processing_thread->join();
    }
  }

  void loadDataset(const std::string &path) {
    // Check the string path to see if it is a rosbag or aprilgrid
    if (path.find(".bag") != std::string::npos) {
      spdlog::debug("Loading rosbag file: {}", path);
      rosbag_files.addFiles(std::vector<std::string>{path});
      spdlog::debug("Size of rosbag_files: {}", rosbag_files.size());
    } else if (path.find(".json") != std::string::npos) {
      aprilgrid_files.addFiles(std::vector<std::string>{path});
      spdlog::debug("Size of aprilgrid_files: {}", aprilgrid_files.size());
    } else {
      std::cerr << "Unknown file type: " << path << std::endl;
    }
  }

//  void feed_images() {
//    // Basically get the image data, and construct the image_data field in the RosbagDataset object
//    // image_data is std::map<int64_t, std::vector<cv::Mat>> image_data;
//
//    for (int64_t t_ns: this->image_timestamps) {
//      std::vector<cv::Mat> cv_img_vec; // One vector for each timestamp
//      const std::vector<ImageData> &img_vec = this->rosbag_io.get()->get_image_data(t_ns);
//
//      for (size_t cam_id = 0; cam_id < this->rosbag_io.get()->get_num_cams(); cam_id++) {
//        const ManagedImage<uint16_t>::Ptr &managed_image_ptr = img_vec[cam_id].img;
//
//        // Check if the managed_image_ptr is valid
//        if (managed_image_ptr) {
//          // Extract the image properties from the managed_image_ptr
//          size_t width = managed_image_ptr->w;
//          size_t height = managed_image_ptr->h;
//          size_t pitch_bytes = managed_image_ptr->pitch;
//
//          // Create a cv::Mat with the appropriate size and data type
//          cv::Mat image_mat_16u(height, width, CV_16U, managed_image_ptr->ptr, pitch_bytes);
//
//          cv::Mat image_mat_8u;
//          image_mat_16u.convertTo(image_mat_8u, CV_8U, 1.0 / 256.0);
//
//          cv_img_vec.push_back(image_mat_8u);
//
//          // Do something with the converted cv::Mat (e.g., perform image processing or display)
//          // ...
//
//          // Note: The cv::Mat `image_mat_8u` contains the scaled-down 8-bit values.
//          // Make sure the ManagedImage remains valid during the lifetime of `image_mat_8u`.
//          // otherwise, the cv::Mat will contain invalid data.
//        }
//      }
//
//      this->image_data.insert({t_ns, cv_img_vec});
//    }
//
//    spdlog::info("Loaded {} timestamps into memory", this->image_timestamps.size());
//  }

  void detectCorners() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }

    if (this->rosbag_files.size() == 0 || this->aprilgrid_files.size() == 0) {
      spdlog::debug("No rosbag or aprilgrid files loaded");
      return;
    }

    // Change this to not even reset the thread when there's cached corners.
    processing_thread = std::make_shared<std::thread>([this]() {
      spdlog::trace("Started detecting corners thread");

      CalibHelper::detectCorners(this->rosbag_files[this->selected], this->aprilgrid_files[this->selected]);
      spdlog::trace("Corner detection is done");
    });
  }

  void detectCheckerboardCorners() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }

    // Don't need aprilgrid json
    if (this->rosbag_files.size() == 0) {
      spdlog::debug("No rosbag files loaded");
      return;
    }

    processing_thread = std::make_shared<std::thread>([this]() {
      spdlog::trace("Started detecting checkerboard corners thread");

      CalibHelper::detectCheckerboardCorners(this->rosbag_files[this->selected], this->checkerboard_params);
      spdlog::trace("Checkerboard corner detection is done");
    });

  }

  void drawCorners() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }

    processing_thread = std::make_shared<std::thread>([this]() {
      spdlog::trace("Started drawing corners");

      for (auto ts: this->rosbag_files[this->selected]->get_image_timestamps()) {
        spdlog::trace("Drawing corners for timestamp: {}", ts);
        std::vector<cv::Mat> &img_vec = this->rosbag_files[this->selected]->image_data.at(ts);

        for (int cam_num = 0; cam_num < this->rosbag_files[this->selected]->get_num_cams(); cam_num++) {
          auto tcid = basalt::TimeCamId(ts, cam_num);
          const CalibCornerData &cr = this->rosbag_files[this->selected]->calib_corners.at(tcid);
//          const CalibCornerData &cr_rej = this->rosbag_files[this->selected]->calib_corners_rejected.at(tcid);

          for (size_t i = 0; i < cr.corners.size(); i++) {
            // The radius is the threshold used for maximum displacement. The search region is slightly larger.
            const float radius = static_cast<float>(cr.radii[i]);
            const Eigen::Vector2d &c = cr.corners[i];

            cv::circle(img_vec[cam_num], cv::Point2d(c[0], c[1]), static_cast<int>(radius),
                       cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
          }

        }
      }
      spdlog::trace("Finished drawing corners");
    });
  }

  void drawCheckerboardCorners() {
    /*
     * https://github.com/pthom/immvision/blob/df657767f08303438c8450e620bf406967cd1dad/src/immvision/internal/drawing/image_drawing.cpp#L23
     * Draw the circles the same way the watched pixels are drawn in immvision.
     * */
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }
    // Draw the checkerboard corners using the cbdetect::Corner struct
    processing_thread = std::make_shared<std::thread>([this]() {
      spdlog::trace("Started drawing checkerboard corners");

      for (auto ts: this->rosbag_files[this->selected]->get_image_timestamps()) {
        spdlog::trace("Drawing corners for timestamp: {}", ts);
        std::vector<cv::Mat> &img_vec = this->rosbag_files[this->selected]->image_data.at(ts);

        for (int cam_num = 0; cam_num < this->rosbag_files[this->selected]->get_num_cams(); cam_num++) {
          auto tcid = basalt::TimeCamId(ts, cam_num);
          const cbdetect::Corner &corners = this->rosbag_files[this->selected]->checkerboard_corners.at(tcid);

          for(int i = 0; i < corners.p.size(); ++i) {
            cv::line(img_vec[cam_num], corners.p[i], corners.p[i] + 20 * corners.v1[i], cv::Scalar(255, 0, 0), 2);
            cv::line(img_vec[cam_num], corners.p[i], corners.p[i] + 20 * corners.v2[i], cv::Scalar(0, 255, 0), 2);
            if(!corners.v3.empty()) {
              cv::line(img_vec[cam_num], corners.p[i], corners.p[i] + 20 * corners.v3[i], cv::Scalar(0, 0, 255), 2);
            }
            cv::circle(img_vec[cam_num], corners.p[i], 3, cv::Scalar(0, 0, 255), -1);
            cv::putText(img_vec[cam_num], std::to_string(i), cv::Point2i(corners.p[i].x - 12, corners.p[i].y - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
          }

        }
      }
      spdlog::trace("Finished drawing corners");
    });
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

//  void drawImageOverlay(cv::Mat image) {
//    int64_t timestamp_ns = this->rosbag_io.get()->get_image_timestamps()[0];
//    TimeCamId tcid(timestamp_ns, 0);
//
//    const CalibCornerData &cr = calib_corners.at(tcid);
//    const CalibCornerData &cr_rej = calib_corners_rejected.at(tcid);
//
//    for (size_t i = 0; i < cr.corners.size(); i++) {
//      // The radius is the threshold used for maximum displacement.
//      // The search region is slightly larger.
//      const float radius = static_cast<float>(cr.radii[i]);
//      const Eigen::Vector2d &c = cr.corners[i];
//
//      cv::circle(image, cv::Point2d(c[0], c[1]), static_cast<int>(radius),
//                 cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
//
//    }
//  }
};