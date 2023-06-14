#pragma once

#include "calibration/calibrator.hpp"
#include "io/aprilgrid_container.h"
#include "io/rosbag_container.h"
#include "recorder/dataset.hpp"

#include "imports.h"
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/plot_corners.h"
#include "ecal_camera/CameraInterface.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

using namespace basalt;

struct AppState {
  RosbagContainer rosbag_files;
  int selected; // selected rosbag_file for display
  AprilGridContainer aprilgrid_files;
  std::shared_ptr<cbdetect::Params> checkerboard_params;
  std::shared_ptr<vk::CameraParams> recorder_params;
  std::shared_ptr<vk::RosbagDatasetRecorder> dataset_recorder;
  basalt::CalibType selectedCalibType; // 0 for AprilGrid, 1 for Checkerboard

  std::map<std::string, uint64_t> num_topics_to_show; // for the rosbag inspector config

  int selectedFrame;

  std::shared_ptr<std::thread> processing_thread;

  ImmVision::ImageParams immvisionParams;

  // Intialize vector of 3 cv::Mat
  std::shared_ptr<std::vector<cv::Mat>> display_imgs = std::make_shared<std::vector<cv::Mat>>(3);

  AppState() {
    spdlog::trace("Initializing AppState object");
    cbdetect::Params cb_params;
    this->checkerboard_params = std::make_shared<cbdetect::Params>(cb_params);
    this->recorder_params = std::make_shared<vk::CameraParams>();
    this->dataset_recorder = std::make_shared<vk::RosbagDatasetRecorder>();
    this->selected = 0; // for the selected rosbag file
    this->selectedFrame = 0;
    this->immvisionParams = ImmVision::ImageParams();
    this->immvisionParams.RefreshImage = true;
    this->selectedCalibType = basalt::CalibType::AprilGrid;

    // Checkerboard params config
    this->checkerboard_params->show_processing = false; // Prints the shit out.
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

  void detectCorners() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }

    spdlog::debug("Selected mode is {}", this->selectedCalibType == basalt::CalibType::AprilGrid ? "AprilGrid" : "Checkerboard");

    switch(this->selectedCalibType) {
      case basalt::CalibType::AprilGrid:
        if (this->rosbag_files.size() == 0 || this->aprilgrid_files.size() == 0) {
          spdlog::debug("No rosbag or aprilgrid files loaded");
          return;
        }
      case basalt::CalibType::Checkerboard:
        if (this->rosbag_files.size() == 0) {
          spdlog::debug("No rosbag files loaded");
          return;
        }
      default:
        break; // Correctly configured
    }

    // Change this to not even reset the thread when there's cached corners.
    processing_thread = std::make_shared<std::thread>([this]() {
      spdlog::trace("Started corner detection worker thread");
      std::shared_ptr<CalibParams> params;
      std::shared_ptr<Calibrator> calibrator;

      switch(this->selectedCalibType) {
        case basalt::CalibType::AprilGrid:
          params = std::make_shared<basalt::AprilGridParams>(this->aprilgrid_files[this->selected]);
          calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selected]);
          break;
        case basalt::CalibType::Checkerboard:
          params = std::make_shared<basalt::CheckerboardParams>(this->checkerboard_params);
          calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selected]);
        default:
          std::runtime_error("Invalid calibration type selected");
          break;
      }

      calibrator->detectCorners(params);
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

//      CalibHelper::detectCheckerboardCorners(this->rosbag_files[this->selected], this->checkerboard_params);
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