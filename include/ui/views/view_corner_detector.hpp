#pragma once

#include "ui/view.hpp"

#include "utils/enum.h"
#include <immvision.h>

#include <thread>

// NOLINTNEXTLINE
BETTER_ENUM(DetectionType, int, Checkerboard, AprilGrid)

class ViewCornerDetector : public View {
public:
    ViewCornerDetector();
    ~ViewCornerDetector() override = default;

    void draw_content() override;

private:
    void draw_config();
    void draw_detection_popup();
    void draw_vkcalibrate_popup();
    void draw_cam_view();

    void detect_corners();
    void draw_corners();
    void launch_vkcalibrate(std::string dataset_path, std::string cb_path,
                                                std::string result_path, std::vector<std::string> cam_types);
    bool show_corners = false;
    bool show_corners_rejected = false;
    int selected_rosbag = 0;
    int selected_frame = 0;
    int selected_aprilgrid = 0;

    // ImmVision Parameters
    ImmVision::ImageParams image_params;

    DetectionType detection_type = DetectionType::Checkerboard;
    // Checkerboard
    int cb_width;
    int cb_height;
    float cb_row_spacing;
    float cb_col_spacing;

    // Flags for OpenCV corner detection
    bool adaptive_thresh;
    bool normalize_image;
    bool filter_quads;
    bool fast_check;
    bool enable_subpix_refine;

    // Member variables pertaining to launching vk_calibrate
    std::vector<std::string> cam_types;
};