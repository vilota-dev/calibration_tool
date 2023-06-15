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
  size_t selectedRosbag = 0; // selected rosbag_file for display
  size_t selectedAprilGrid = 0;
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
    this->selectedFrame = 0;
    this->immvisionParams = ImmVision::ImageParams();
    this->immvisionParams.RefreshImage = true;
    this->selectedCalibType = basalt::CalibType::Checkerboard;

    // Checkerboard params config
    this->checkerboard_params->show_processing = false; // Prints the shit out.

    // Load the commonly used april grids
    basalt::AprilGridPtr g1 = std::make_shared<basalt::AprilGrid>(7, 4, 0.0946, 0.3, 0, "16h5");
    std::vector<basalt::AprilGridPtr> commonGrids = { g1 };
    this->aprilgrid_files.addFiles(commonGrids);
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
      spdlog::debug("{}", path);
      basalt::AprilGridPtr grid = std::make_shared<basalt::AprilGrid>(path);
      aprilgrid_files.addFiles(std::vector<basalt::AprilGridPtr>{grid});
      spdlog::debug("Size of aprilgrid_files: {}", aprilgrid_files.size());
    } else {
      spdlog::error("Unknown file type: {}", path);
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
          params = std::make_shared<basalt::AprilGridParams>(this->aprilgrid_files[this->selectedAprilGrid]);
          calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selectedRosbag]);
          break;
        case basalt::CalibType::Checkerboard:
          params = std::make_shared<basalt::CheckerboardParams>(this->checkerboard_params);
          calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selectedRosbag]);
        default:
          std::runtime_error("Invalid calibration type selected");
          break;
      }

      calibrator->detectCorners(params);
    });
  }

  void drawCorners() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }

    processing_thread = std::make_shared<std::thread>([this]() {
      spdlog::trace("Started drawing corners");

      for (auto ts: this->rosbag_files[this->selectedRosbag]->get_image_timestamps()) {
        spdlog::trace("Drawing corners for timestamp: {}", ts);
        std::vector<cv::Mat> &img_vec = this->rosbag_files[this->selectedRosbag]->image_data.at(ts);

        for (int cam_num = 0; cam_num < this->rosbag_files[this->selectedRosbag]->get_num_cams(); cam_num++) {
          auto tcid = basalt::TimeCamId(ts, cam_num);
          const CalibCornerData &cr = this->rosbag_files[this->selectedRosbag]->calib_corners.at(tcid);
//          const CalibCornerData &cr_rej = this->rosbag_files[this->selected]->calib_corners_rejected.at(tcid);

          for (size_t i = 0; i < cr.corners.size(); i++) {
            // The radius is the threshold used for maximum displacement. The search region is slightly larger.
            const float radius = static_cast<float>(cr.radii[i]);
            const Eigen::Vector2d &c = cr.corners[i];
            const auto idx = cr.corner_ids[i];

            cv::circle(img_vec[cam_num], cv::Point2d(c[0], c[1]), static_cast<int>(radius),
                       cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

            cv::putText(img_vec[cam_num], std::to_string(idx), cv::Point2i(c[0] - 12, c[1] - 6),
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
};