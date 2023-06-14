#include "imgui.h"
#include "gui/imports.h"

void img_display(ImmVision::ImageParams &image_params, AppState &app_state) {
    if (!ImGui::Begin("Image Inspector")) {
      ImGui::End();
      return;
    }
    ImGui::SliderInt("Selected Frame", &app_state.selectedFrame, 0, app_state.rosbag_files[app_state.selected]->get_image_timestamps().size() - 1);
    ImGui::NewLine();

    auto ts = app_state.rosbag_files[app_state.selected]->get_image_timestamps()[app_state.selectedFrame];
    auto img_to_display = app_state.rosbag_files[app_state.selected]->image_data.at(ts);

    static int threshold = 1;
    float childWidth = ImGui::GetWindowWidth() / 3.0f;

    image_params.ImageDisplaySize = cv::Size(childWidth, 0);
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.95f);

    for (size_t i = 0; i < app_state.rosbag_files[app_state.selected]->get_num_cams(); i++) {
      ImmVision::Image(std::string("cam_") + std::to_string(i), img_to_display[i], &image_params);
      ImGui::SameLine();
    }

    ImGui::End();
}