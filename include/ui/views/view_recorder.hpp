#pragma once

#include "ui/view.hpp"
#include "recorder/dataset.hpp"
#include "recorder/presets.hpp"

#ifndef IMMVISION_API
#include <immvision.h>
#endif

#include <vector>
#include <optional>

class ViewRecorder : public View {
public:
    ViewRecorder();
    ~ViewRecorder() override = default;

    void draw_content() override;
    void draw_controls();
    void draw_cam_view();
private:
    int custom_camera; // whether or not we want to use presets or custom camera
    vk::RecordMode selected_mode; // snapshot or continuous
    vk::Preset selected_preset;
    vk::RosbagDatasetRecorder dataset_recorder;
    std::shared_ptr<std::unordered_map<std::string, cv::Mat>> display_imgs; // vector of images to display, live view
    ImmVision::ImageParams display_params; // parameters for the ImmVision API
};