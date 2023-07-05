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
        ImVec2 button_size = ImGui::CalcItemSize(ImVec2{300, 0}, 0.0f, 0.0f);

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

        {
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

            ImGui::Checkbox("Show corners", &this->show_corners); ImGui::SameLine();
            ImGui::Checkbox("Show corners rejected", &this->show_corners_rejected); ImGui::SameLine();

            if (ImGui::Button("Detect Corners")) {
                ImGui::OpenPopup("Detection Config");
            } ImGui::SameLine();
            if (ImGui::Button("Draw Corners")) {
                spdlog::trace("Starting corner drawing");
                // this->draw_corners();
            }
        }
    }

    // Open the popup if the button is clicked.
    this->draw_popup();

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
            });
            break;
        }
    }
}

void ViewCornerDetector::draw_corners() {

}