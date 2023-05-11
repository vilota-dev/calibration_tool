#ifndef GUI_HPP
#define GUI_HPP

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "immvision.h"
#include <glad/glad.h> // Initialize with gladLoadGL()
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include "spdlog/spdlog.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "basalt/calibration/calibration.hpp"
#include <cereal/types/unordered_map.hpp>

// Std library imports
#include <cstdio>
#include <filesystem>
#include <cstdlib>

enum class Orientation {
    Horizontal,
    Vertical
};

struct SobelParams {
    float blur_size = 1.25f;
    int deriv_order = 1;  // order of the derivative
    int k_size = 7;  // size of the extended Sobel kernel it must be 1, 3, 5, or 7 (or -1 for Scharr)
    Orientation orientation = Orientation::Vertical;
};

std::string ResourcesDir();
cv::Mat ComputeSobel(const cv::Mat &image, const SobelParams &params);
bool GuiSobelParams(SobelParams &params);

struct AppState {
    cv::Mat image;
    cv::Mat imageSobel;
    SobelParams sobelParams;

    ImmVision::ImageParams immvisionParams;
    ImmVision::ImageParams immvisionParamsSobel;

    AppState(const std::string &image_file) {
        image = cv::imread(image_file);
        sobelParams = SobelParams();
        imageSobel = ComputeSobel(image, sobelParams);

        immvisionParams = ImmVision::ImageParams();
        immvisionParams.ImageDisplaySize = cv::Size(300, 0);
        immvisionParams.ZoomKey = "z";

        immvisionParamsSobel = ImmVision::ImageParams();
        immvisionParamsSobel.ImageDisplaySize = cv::Size(600, 0);
        immvisionParamsSobel.ZoomKey = "z";
        immvisionParamsSobel.ShowOptionsPanel = true;
    }
};

void calibrate_gui();
void run_gui();

#endif // GUI_HPP