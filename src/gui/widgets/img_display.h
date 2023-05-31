#include "imgui.h"
#include "imports.h"

void img_display(ImmVision::ImageParams &image_params, std::vector<cv::Mat> &images) {
    if (!ImGui::Begin("Image Inspector")) {
      ImGui::End();
      return;
    }
    static int threshold = 1;
    float childWidth = ImGui::GetWindowWidth() / 3.0f;

    image_params.ImageDisplaySize = cv::Size(childWidth, 0);
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.95f);

    for (size_t i = 0; i < 3; i++) {
      ImmVision::Image(std::string("cam_") + std::to_string(i), images[0], &image_params);
      ImGui::SameLine();
    }

    ImGui::NewLine();

    ImGui::SliderInt("Gaussian Blur", &threshold, 0, 101);

    ImGui::SliderInt("Frame Number", &threshold, 0, 100, "%.0f", 1.0f);


    ImGui::End();
}