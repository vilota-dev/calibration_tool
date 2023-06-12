#include "imgui.h"
#include "gui/imports.h"

void draw_live_view(ImmVision::ImageParams &image_params, std::shared_ptr<std::vector<cv::Mat>> &display_imgs) {
    if (!ImGui::Begin("Live Camera View")) {
        ImGui::End();
        return;
    }

    float childWidth = ImGui::GetWindowWidth() / 3.0f;
    image_params.ImageDisplaySize = cv::Size(childWidth, 0);

    for (size_t i = 0; i < display_imgs->size(); i++) {
        ImmVision::Image(std::string("cam_") + std::to_string(i), display_imgs->at(i), &image_params);
        ImGui::SameLine();
    }

    ImGui::End();
}