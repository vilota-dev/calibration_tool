#pragma once

#include "calibration/calibrator.hpp"
#include "io/rosbag_container.h"

#include "BS_thread_pool.hpp"

#include <memory>
#include <string>

/*
 * Meyers Singleton class to hold the state of the application with thread-safe initialization.
 * Provide global access to the application state instance.
 * */
class AppState {
private:
    // Constructor and destructor should be private
    AppState() : rosbag_files(), selected_bag(0), thread_pool(std::thread::hardware_concurrency() - 1) {}
    ~AppState() = default;

    RosbagContainer rosbag_files;
    int selected_bag = 0; // selected rosbag_file for display

protected:
    BS::thread_pool thread_pool;

public:
    // Delete copy and move constructors and assign operators
    AppState(AppState const&) = delete;             // Copy construct
    AppState(AppState&&) = delete;                  // Move construct
    AppState& operator=(AppState const&) = delete;  // Copy assign
    AppState& operator=(AppState &&) = delete;      // Move assign

    static AppState& get_instance() {
        static AppState instance;
        return instance;
    }

    /*
     * Push tasks to thread pool, does not return std::future
     * */
    template <typename F, typename... A>
    static void submit_task(F&& task, A&&... args) {
        auto &app_state = AppState::get_instance();
        app_state.thread_pool.push_task(std::forward<F>(task), std::forward<A>(args)...);

    }

    void load_dataset();
};


//void AppState::detect_corners() {
//  if (processing_thread) {
//    processing_thread->join();
//    processing_thread.reset();
//  }
//
//  spdlog::debug("Selected mode is {}", this->selectedCalibType == basalt::CalibType::AprilGrid ? "AprilGrid" : "Checkerboard");
//
//  // Change this to not even reset the thread when there's cached corners.
//  processing_thread = std::make_shared<std::thread>([this]() {
//      spdlog::trace("Started corner detection worker thread");
//      std::shared_ptr<CalibParams> params;
//      std::shared_ptr<Calibrator> calibrator;
//
//      switch(this->selectedCalibType) {
//          case basalt::CalibType::AprilGrid:
//              params = std::make_shared<basalt::AprilGridParams>(this->aprilgrid_files[this->selectedAprilGrid]);
//              calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selectedRosbag]);
//              break;
//          case basalt::CalibType::Checkerboard_CBDETECT:
//              params = std::make_shared<basalt::CBCheckerboardParams>(this->checkerboard_params);
//              calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selectedRosbag]);
//              break;
//          case basalt::CalibType::Checkerboard_OpenCV:
//              params = std::make_shared<basalt::OpenCVCheckerboardParams>(*this->opencv_checkerboard_params);
//              calibrator = std::make_unique<basalt::Calibrator>(this->rosbag_files[this->selectedRosbag]);
//              break;
//          default:
//              spdlog::error("Invalid calibration type selected");
//              std::exit(1);
//      }
//
//      calibrator->detectCorners(params);
//  });
//}
//
//void AppState::draw_corners() {
//  if (processing_thread) {
//    processing_thread->join();
//    processing_thread.reset();
//  }
//
//  processing_thread = std::make_shared<std::thread>([this]() {
//      spdlog::trace("Started drawing corners");
//
//      for (auto ts: this->rosbag_files[this->selectedRosbag]->get_image_timestamps()) {
//          spdlog::trace("Drawing corners for timestamp: {}", ts);
//          std::vector<cv::Mat> &img_vec = this->rosbag_files[this->selectedRosbag]->image_data.at(ts);
//
//          for (int cam_num = 0; cam_num < this->rosbag_files[this->selectedRosbag]->get_num_cams(); cam_num++) {
//              auto tcid = basalt::TimeCamId(ts, cam_num);
//              const CalibCornerData &cr = this->rosbag_files[this->selectedRosbag]->calib_corners.at(tcid);
//              //          const CalibCornerData &cr_rej = this->rosbag_files[this->selected]->calib_corners_rejected.at(tcid);
//
//              for (size_t i = 0; i < cr.corners.size(); i++) {
//                  // The radius is the threshold used for maximum displacement. The search region is slightly larger.
//                  const float radius = static_cast<float>(cr.radii[i]);
//                  const Eigen::Vector2d &c = cr.corners[i];
//                  // Convert to cv::Point2f for opencv drawing
//                  std::vector<cv::Point2f> cv_corners;
//                  for (const auto &corner : cr.corners) {
//                      cv_corners.emplace_back(corner[0], corner[1]);
//                  }
//                  const auto idx = cr.corner_ids[i];
//
//                  if (this->selectedCalibType == basalt::CalibType::Checkerboard_OpenCV) {
//                      cv::Size size(this->opencv_checkerboard_params->width, this->opencv_checkerboard_params->height);
//                      cv::drawChessboardCorners(img_vec[cam_num], size, cv::Mat(cv_corners), true);
//                  } else {
//                      cv::circle(img_vec[cam_num], cv::Point2d(c[0], c[1]), static_cast<int>(radius),
//                                 cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
//
//                      cv::putText(img_vec[cam_num], std::to_string(idx), cv::Point2i(c[0] - 12, c[1] - 6),
//                                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
//                  }
//              }
//
//          }
//      }
//      spdlog::trace("Finished drawing corners");
//  });
//}