#pragma once

#include "ui/view.hpp"

#include "utils/enum.h"
#include <immvision.h>

// NOLINTNEXTLINE
BETTER_ENUM(DetectionType, int, Checkerboard, AprilGrid)

class ViewCornerDetector : public View {
public:
    ViewCornerDetector();
    ~ViewCornerDetector() override = default;

    void draw_content() override;

private:
    void draw_popup();
    static void detect_corners(DetectionType detection_type);
    void draw_corners();

    bool show_corners = false;
    bool show_corners_rejected = false;
    int selected_rosbag = 0;
    int selected_frame = 0;

    // ImmVision Parameters
    ImmVision::ImageParams image_params;

    DetectionType detection_type = DetectionType::Checkerboard;
    // Checkerboard
    int cb_width;
    int cb_height;
    bool adaptive_thresh;
    bool normalize_image;
    bool filter_quads;
    bool fast_check;
    bool enable_subpix_refine;
};