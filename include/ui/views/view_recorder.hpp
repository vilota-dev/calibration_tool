#pragma once

#include "ui/view.hpp"
#include "recorder/dataset.hpp"
#include "recorder/presets.hpp"

#ifndef IMMVISION_API
#include <immvision.h>
#endif

#include <vector>

class ViewRecorder : public View {
public:
    ViewRecorder();
    ~ViewRecorder() override = default;

    void draw_content() override;
    void draw_controls();
    void draw_cam_view();
private:
    int custom_camera;
    vk::CameraParams recorder_params;
    vk::RecordMode selected_mode;
    vk::RosbagDatasetRecorder dataset_recorder;
    std::shared_ptr<std::vector<cv::Mat>> display_imgs;
    ImmVision::ImageParams display_params;
};