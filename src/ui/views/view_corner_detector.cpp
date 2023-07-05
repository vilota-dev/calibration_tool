#include "ui/views/view_corner_detector.hpp"

#include "app_state.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include <spdlog/spdlog.h>

ViewCornerDetector::ViewCornerDetector() : View("Corner Detector") {
    this->cb_width = 8;
    this->cb_height = 6;
    this->adaptive_thresh = true;
    this->normalize_image = true;
    this->filter_quads = true;
    this->fast_check = true;
    this->enable_subpix_refine = true;

    this->image_params.RefreshImage = true;
}

void ViewCornerDetector::draw_config() {
    ImGui::BeginChild("Corner Detector Config", ImVec2(0, 50), true);

    auto &app_state = AppState::get_instance();

    if (ImGui::BeginCombo("ROS .bag Datasets",
                          app_state.rosbag_files[this->selected_rosbag]->get_file_path().c_str())) {
        for (int i = 0; i < app_state.rosbag_files.size(); i++) {
            bool is_selected = (this->selected_rosbag == i);
            if (ImGui::Selectable(app_state.rosbag_files[i]->get_file_path().c_str(), is_selected)) {
                this->selected_rosbag = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Checkbox("Show corners", &this->show_corners); ImGui::SameLine();
    ImGui::Checkbox("Show corners rejected", &this->show_corners_rejected); ImGui::SameLine();

    if (ImGui::Button("Detect Corners")) {
        ImGui::OpenPopup("Detection Config");
    }

    // Open the popup if the button is clicked.
    this->draw_popup();

    ImGui::EndChild();
}

void ViewCornerDetector::draw_cam_view() {
    ImGui::BeginChild("Camera Views", ImVec2(0, 0), true);

    auto &app_state = AppState::get_instance();

    // Get the selected rosbag
    auto &rosbag = app_state.rosbag_files[this->selected_rosbag];

    // Draw the image slider and ImmVision
    ImGui::SliderInt("Selected Frame", &selected_frame, 0, rosbag->get_image_timestamps().size() - 1);
    ImGui::NewLine();

    auto ts = rosbag->get_image_timestamps()[this->selected_frame];
    auto img_to_display = rosbag->image_data.at(ts);

    auto num_cams = static_cast<int>(rosbag->get_num_cams());

    ImGui::Columns(num_cams);

    int idx = 0;
    for (auto &cam_name : rosbag->get_camera_names()) {
        // Quick hack to ensure each immvision object has a different zoom key without
        // creating multiple instances of the ImageParams object (HACK DOESN'T WORK)
        this->image_params.ZoomKey = cam_name;
        ImmVision::Image(cam_name, img_to_display[idx], &this->image_params);
        ImGui::NextColumn();
        idx++;
    }

    ImGui::EndChild();
}

void ViewCornerDetector::draw_content() {
    if (!ImGui::Begin(this->get_name().c_str())) {
        ImGui::End();
        return;
    }

    auto &app_state = AppState::get_instance();

    /*
     * If there's no ROS .bag loaded, then we hide everything.
     * */
    if (app_state.rosbag_files.size() == 0) {
        // Note that 0 tells ImGui to "just use the default"
        ImVec2 button_size = ImGui::CalcItemSize(ImVec2{300, 50}, 0.0f, 0.0f);

        ImVec2 avail = ImGui::GetWindowSize();

        // Calculate center of window for button.
        ImVec2 centre_position_for_button{
            // we have two widgets, so twice the size - and we need to account for the spacing in the middle
                (avail.x - button_size.x) / 2,
                (avail.y - button_size.y) / 2
        };

        // tell ImGui to render the button at the new position
        ImGui::SetCursorPos(centre_position_for_button);

        if (ImGui::Button("Load ROS .bag dataset to continue", button_size)) {
            app_state.load_dataset();
        }
    } else {
        this->draw_config();
        this->draw_cam_view();
    }

    ImGui::End();
}

void ViewCornerDetector::draw_popup() {
    auto &app_state = AppState::get_instance();

    if (ImGui::BeginPopupModal("Detection Config", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (ImGui::BeginCombo("corner_detection_type", this->detection_type._to_string())) {
            for (DetectionType type: DetectionType::_values()) {
                bool is_selected = (detection_type == type);
                if (ImGui::Selectable(type._to_string(), is_selected)) {
                    detection_type = type;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        /*
         * Draw calibration board specific details.
         * libcbdetect is removed for now, since we are not using it. Refer to previous commits if you want to use it.
         * */
        switch (detection_type) {
            case DetectionType::AprilGrid: {
                if (ImGui::Button("Load AprilGrid")) {
                    app_state.load_aprilgrid();
                }
                if (app_state.aprilgrid_files.size() != 0) {
                    ImGui::SameLine();
                    const char *currentSelection = app_state.aprilgrid_files[this->selected_aprilgrid]->get_name().c_str();

                    if (ImGui::BeginCombo("April Grid", currentSelection)) {
                        for (int i = 0; i < app_state.aprilgrid_files.size(); i++) {
                            bool isSelected = (currentSelection == app_state.aprilgrid_files[i]->get_name().c_str());
                            if (ImGui::Selectable(app_state.aprilgrid_files[i]->get_name().c_str(), isSelected)) {
                                this->selected_aprilgrid = i;
                            }
                            if (isSelected) {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }

                    ImGui::Text("tagRows: %i", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagRows());
                    ImGui::Text("tagCols: %i", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagCols());
                    ImGui::Text("tagSize: %f", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagSize());
                    ImGui::Text("tagSpacing: %f", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagSpacing());
                    ImGui::Text("lowID: %i", app_state.aprilgrid_files[this->selected_aprilgrid]->getLowId());
                    ImGui::Text("tagFamily: %s", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagFamily().c_str());
                }
                break;
            }
            case DetectionType::Checkerboard: {
                ImGui::InputInt("Rows", &this->cb_width);
                ImGui::InputInt("Cols", &this->cb_height);
                // Flags
                ImGui::Checkbox("Adaptive Threshold", &this->adaptive_thresh);
                ImGui::Checkbox("Normalize Image", &this->normalize_image);
                ImGui::Checkbox("Filter Quads", &this->filter_quads);
                ImGui::Checkbox("Fast Check", &this->fast_check);
                ImGui::Checkbox("Enable sub-pixel refinement",
                                &this->enable_subpix_refine);
                break;
            }
        }

        if (ImGui::Button("Detect!", ImVec2(120, 0))) {
            this->detect_corners();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();

        if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }

        ImGui::EndPopup();
    }
}

void ViewCornerDetector::detect_corners() {
    spdlog::debug("Detecting corners with {} mode", detection_type._to_string());

    switch (this->detection_type) {
        case DetectionType::AprilGrid: {
            //NOLINTNEXTLINE
            AppState::get_instance().submit_task([this]() {
                auto &app_state = AppState::get_instance();
                auto params = std::make_shared<basalt::AprilGridParams>(
                        app_state.aprilgrid_files[this->selected_aprilgrid]);
                auto calibrator = std::make_unique<basalt::Calibrator>(
                        app_state.rosbag_files[this->selected_rosbag]);

                calibrator->detectCorners(params);
                this->draw_corners();
            });
            break;
        }
        case DetectionType::Checkerboard: {
            //NOLINTNEXTLINE
            AppState::get_instance().submit_task([this]() {
                auto &app_state = AppState::get_instance();
                auto params = std::make_shared<basalt::OpenCVCheckerboardParams>(
                        this->cb_width, this->cb_height, this->adaptive_thresh, this->normalize_image,
                        this->filter_quads, this->fast_check, this->enable_subpix_refine);
                auto calibrator = std::make_unique<basalt::Calibrator>(
                        app_state.rosbag_files[this->selected_rosbag]);

                calibrator->detectCorners(params);
                this->draw_corners();
            });
            break;
        }
    }

}

void ViewCornerDetector::draw_corners() {
    //NOLINTNEXTLINE
    AppState::get_instance().submit_task([this]() {
        spdlog::trace("Started drawing corners");
        auto &app_state = AppState::get_instance();

        for (auto ts: app_state.rosbag_files[this->selected_rosbag]->get_image_timestamps()) {
            spdlog::trace("Drawing corners for timestamp: {}", ts);
            std::vector<cv::Mat> &img_vec = app_state.rosbag_files[this->selected_rosbag]->image_data.at(ts);
            spdlog::trace("Doesn't reach here");

            for (int cam_num = 0; cam_num < app_state.rosbag_files[this->selected_rosbag]->get_num_cams(); cam_num++) {
                auto tcid = basalt::TimeCamId(ts, cam_num);
                const basalt::CalibCornerData &cr = app_state.rosbag_files[this->selected_rosbag]->calib_corners.at(tcid);
//          const CalibCornerData &cr_rej = this->rosbag_files[this->selected]->calib_corners_rejected.at(tcid);

                for (size_t i = 0; i < cr.corners.size(); i++) {
                    // The radius is the threshold used for maximum displacement. The search region is slightly larger.
                    const float radius = static_cast<float>(cr.radii[i]);
                    const Eigen::Vector2d &c = cr.corners[i];
                    // Convert to cv::Point2f for opencv drawing
                    std::vector<cv::Point2f> cv_corners;
                    for (const auto &corner : cr.corners) {
                        cv_corners.emplace_back(corner[0], corner[1]);
                    }
                    const auto idx = cr.corner_ids[i];

                    switch (this->detection_type) {
                        case DetectionType::AprilGrid: {
                            cv::circle(img_vec[cam_num], cv::Point2d(c[0], c[1]), static_cast<int>(radius),
                                       cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

                            cv::putText(img_vec[cam_num], std::to_string(idx), cv::Point2i(c[0] - 12, c[1] - 6),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                            break;
                        }
                        case DetectionType::Checkerboard: {
                            cv::Size size(this->cb_width, this->cb_height);
                            cv::drawChessboardCorners(img_vec[cam_num], size, cv::Mat(cv_corners), true);
                            break;
                        }
                    }
                }

            }
        }
        spdlog::trace("Finished drawing corners");
    });
}