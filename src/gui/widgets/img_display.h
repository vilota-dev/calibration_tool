#include "imgui.h"
#include "gui/imports.h"
#include "utils/enum.h"

BETTER_ENUM(DetectionType, int, Checkerboard, AprilGrid)

// forward declare some functions
void draw_detection_config_popup();
void detect_corners();

void draw_corner_detection() {
    if (!ImGui::Begin("Corner Detection")) {
      ImGui::End();
      return;
    }

    // Initialize static variables
    static bool show_corners = false;
    static bool show_corners_rejected = false;
    static int selected_frame = 0;

    ImGui::Checkbox("Show corners", &show_corners); ImGui::SameLine();
    ImGui::Checkbox("Show corners rejected", &show_corners_rejected); ImGui::SameLine();

    if (ImGui::Button("Detect Corners")) {
      ImGui::OpenPopup("Detection Config");
//      app_state.detectCorners();
    }
    ImGui::SameLine();
    if (ImGui::Button("Draw Corners")) {
      spdlog::trace("Starting corner drawing");
//      app_state.drawCorners();
    }

    draw_detection_config_popup();

//    ImGui::SliderInt("Selected Frame", &selected_frame, 0, app_state.rosbag_files[app_state.selectedRosbag]->get_image_timestamps().size() - 1);
//    ImGui::NewLine();
//
//    auto ts = app_state.rosbag_files[app_state.selectedRosbag]->get_image_timestamps()[app_state.selectedFrame];
//    auto img_to_display = app_state.rosbag_files[app_state.selectedRosbag]->image_data.at(ts);
//
//    static int threshold = 1;
//    float childWidth = ImGui::GetWindowWidth() / 3.0f;
//
//    image_params.ImageDisplaySize = cv::Size(childWidth, 0);
//    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.95f);
//
//    for (size_t i = 0; i < app_state.rosbag_files[app_state.selectedRosbag]->get_num_cams(); i++) {
//      ImmVision::Image(std::string("cam_") + std::to_string(i), img_to_display[i], &image_params);
//      ImGui::SameLine();
//    }

    ImGui::End();
}

void detect_corners(DetectionType detection_type) {
    spdlog::debug("Detecting corners with {}", detection_type._to_string());
    switch (detection_type) {
        case DetectionType::Checkerboard:
            break;
        case DetectionType::AprilGrid:
            spdlog::debug("Detecting corners with AprilGrid");
            break;
    }
}

void draw_detection_config_popup() {
    static DetectionType detection_type = DetectionType::Checkerboard;

    if (ImGui::BeginPopupModal("Detection Config", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
      if (ImGui::BeginCombo("corner_detection_type", detection_type._to_string())) {
            for (DetectionType type : DetectionType::_values()) {
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

      if (ImGui::Button("Detect!", ImVec2(120, 0))) {
//          detect_corners();
          ImGui::CloseCurrentPopup();
      }
      ImGui::SetItemDefaultFocus();
      ImGui::SameLine();

      if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }

      ImGui::EndPopup();
    }
}