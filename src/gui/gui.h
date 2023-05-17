#pragma once

#include "io/dataset_io.h"
#include "app_state.h"
#include "corner_detection/detect_corner.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "immvision.h"
#include <glad/glad.h> // Initialize with gladLoadGL()
#include <GLFW/glfw3.h> // Will drag system OpenGL headerss

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "spdlog/spdlog.h"
#include <nfd.h>

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

std::string ResourcesDir() {
  std::filesystem::path root(PROJECT_ROOT); // use cmake preprocessor macros
  return (root / "data").string(); // Need to change this
}

cv::Mat ComputeSobel(const cv::Mat &image, const SobelParams &params) {
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat img_float;
  gray.convertTo(img_float, CV_32F, 1.0 / 255.0);
  cv::Mat blurred;
  cv::GaussianBlur(img_float, blurred, cv::Size(), params.blur_size, params.blur_size);

  double good_scale = 1.0 / std::pow(2.0, (params.k_size - 2 * params.deriv_order - 2));

  int dx, dy;
  if (params.orientation == Orientation::Vertical) {
    dx = params.deriv_order;
    dy = 0;
  } else {
    dx = 0;
    dy = params.deriv_order;
  }
  cv::Mat r;
  cv::Sobel(blurred, r, CV_64F, dx, dy, params.k_size, good_scale);
  return r;
}

bool GuiSobelParams(SobelParams &params) {
  bool changed = false;

  // Blur size
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
  if (ImGui::SliderFloat("Blur size", &params.blur_size, 0.5f, 10.0f)) {
    changed = true;
  }
  ImGui::SameLine();
  ImGui::Text(" | ");
  ImGui::SameLine();

  // Deriv order
  ImGui::Text("Deriv order");
  ImGui::SameLine();
  for (int deriv_order = 1; deriv_order <= 4; ++deriv_order) {
    if (ImGui::RadioButton(std::to_string(deriv_order).c_str(), params.deriv_order == deriv_order)) {
      changed = true;
      params.deriv_order = deriv_order;
    }
    ImGui::SameLine();
  }

  ImGui::Text(" | ");
  ImGui::SameLine();

  ImGui::Text("Orientation");
  ImGui::SameLine();
  if (ImGui::RadioButton("Horizontal", params.orientation == Orientation::Horizontal)) {
    changed = true;
    params.orientation = Orientation::Horizontal;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Vertical", params.orientation == Orientation::Vertical)) {
    changed = true;
    params.orientation = Orientation::Vertical;
  }

  return changed;
}

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

#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif

void config_gui() {
  ImGui::Begin("Calibration GUI");

  // Make a slider for a slider for the frame
  static int frame_slider = 0;
  ImGui::SliderInt("Frame", &frame_slider, 0, 100);

  static bool show_corners = false;
  ImGui::Checkbox("Show corners", &show_corners);

  static bool show_corners_rejected = false;
  ImGui::Checkbox("Show corners rejected", &show_corners_rejected);

  // make a slider int
  static int threshold = 0;
  ImGui::SliderInt("Gaussian Blur", &threshold, 0, 101);

  if (ImGui::Button("Load Dataset")) {
    spdlog::info("Loading dataset");
    // Insert the NFDE stuff here
  }

  if (ImGui::Button("Detect Corners")) {
    spdlog::info("Starting corner detection");
    // Insert the NFDE stuff here
  }

  if (ImGui::Button("Optimize")) {
    spdlog::info("Starting calibration");
    // Optimize function
  }

  ImGui::End();

}

void calibrate_gui() {
  static AppState appState(ResourcesDir() + "/lenna.png");

  ImGui::TextWrapped(R"(
        This example shows a example of image processing (sobel filter) where you can adjust the params and see their effect in real time.

        Apply Colormaps to the filtered image in the options tab.
    )");
  ImGui::Separator();

  if (GuiSobelParams(appState.sobelParams)) {
    appState.imageSobel = ComputeSobel(appState.image, appState.sobelParams);
    appState.immvisionParamsSobel.RefreshImage = true;
  }
  ImmVision::Image("Original", appState.image, &appState.immvisionParams);
  ImmVision::Image("Deriv", appState.imageSobel, &appState.immvisionParamsSobel);
}

void draw_main_menu_bar(AppState1 &data) {
  static nfdchar_t *outPath;
  static nfdfilteritem_t filterItem[1] = {{"ROS .bag file", "bag"}};

  ImGui::BeginMainMenuBar();
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Load .bag file")) {
      nfdresult_t result = NFD_OpenDialog(&outPath, filterItem, 1, NULL);
      if (result == NFD_OKAY) {
        spdlog::info("Success!");
        spdlog::info("File is located here: {}", outPath);
        data.loadDataset(outPath);
        NFD_FreePath(outPath);
      } else if (result == NFD_CANCEL) {
        spdlog::info("User pressed cancel.");
      } else {
        spdlog::error("Error: {}", NFD_GetError());
      }
    }
    ImGui::EndMenu();
  }

  ImGui::EndMainMenuBar();
}

static void glfw_error_callback(int error, const char *description) {
  spdlog::error("GLFW Error {}: {}", error, description);
}

void run_gui() {
  spdlog::info("Starting GUI");

  AppState1 app_state("/Users/tejas/Developer/vilota-dev/calibration_tool/data/2023-04-21/aprilgrid_6x6_15.5percent.json");
  app_state.loadDataset("/Users/tejas/Developer/vilota-dev/calibration_tool/data/2023-04-21/run1/2023-04-21-vk180-calib-run1.bag");

  // Check if calib corners is empty
  if (app_state.calib_corners.empty()) {
    spdlog::info("Calib corners is empty, starting corner detection");
    detectCorners(app_state.rosbag_io.get_data(), app_state.april_grid, app_state.calib_corners, app_state.calib_corners_rejected);
  } else {
    spdlog::info("Cache file found, skipping corner detection");
  }

  const std::vector<basalt::ImageData> &img_vec = app_state.rosbag_io.get_data()->get_image_data(633071364884620);
  cv::Mat image = app_state.convert(img_vec[0].img);
  int64_t timestamp_ns = app_state.rosbag_io.get_data()->get_image_timestamps()[0];
  TimeCamId tcid(633071364884620, 0);

  const CalibCornerData &cr = app_state.calib_corners.at(tcid);
  const CalibCornerData &cr_rej = app_state.calib_corners_rejected.at(tcid);

  for (size_t i = 0; i < cr.corners.size(); i++) {
    // The radius is the threshold used for maximum displacement.
    // The search region is slightly larger.
    const float radius = static_cast<float>(cr.radii[i]);
    const Eigen::Vector2d& c = cr.corners[i];

    cv::circle(image, cv::Point2d(c[0], c[1]), static_cast<int>(radius),
               cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  }

  // Initialize Native file dialog
  NFD_Init();
  nfdchar_t *outPath;
  nfdresult_t result;

  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
    spdlog::error("Failed to initialize GLFW");
    std::exit(EXIT_FAILURE);
  }

  // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
  // GL ES 2.0 + GLSL 100
  const char* glsl_version = "#version 100";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
  // GL 3.2 + GLSL 150
  const char *glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

  // Create window with graphics context
  GLFWwindow *window = glfwCreateWindow(1280, 720, "Calibration Tool", nullptr, nullptr);
  if (window == nullptr) {
    spdlog::error("Failed to create GLFW window");
    glfwTerminate();
    std::exit(1);
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  // Load Glad
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    spdlog::error("Failed to initialize GLAD");
    std::exit(EXIT_FAILURE);
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void) io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows
  //io.ConfigViewportsNoAutoMerge = true;
  //io.ConfigViewportsNoTaskBarIcon = true;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  //ImGui::StyleColorsLight();

  // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
  ImGuiStyle &style = ImGui::GetStyle();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    style.WindowRounding = 0.0f;
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
  }

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Our state
  bool show_demo_window = true;
  //bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // For Dockspace stuff
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

    // Show Demo window
    ImGui::ShowDemoWindow(&show_demo_window);

    config_gui();
    calibrate_gui();

    draw_main_menu_bar(app_state);

    static ROSbag bag;

    if (ImGui::Begin("ROS .bag Inspector", nullptr, ImGuiWindowFlags_MenuBar)) {
      static nfdfilteritem_t filterItem[1] = {{"ROS .bag file", "bag"}};

      draw_menu_bar(bag, result, outPath);
      int selected = draw_files_left_panel(0);

      ImGui::SameLine();

      draw_bag_content(bag, selected);

      // Draw a button to call the load image function
      if (ImGui::Button("Load Image")) {
        // Print values of the image[0] cv::Mat using spdlog
      }
    }
    ImGui::End();

    if (ImGui::Begin("Image Display")) {

      ImmVision::ImageDisplay("Fk", image); //img_vec[0].img.get()
      ImmVision::ImageDisplay("Fk2", app_state.convert(img_vec[0].img)); //img_vec[0].img.get()
      ImmVision::ImageDisplay("Fk3", app_state.convert(img_vec[2].img)); //img_vec[0].img.get()

//      ImmVision::ImageDisplay("Cat", app_state.image_data.at(app_state.image_timestamps[1])[0]);
//      ImmVision::ImageDisplay("Dog", app_state.image_data.at(app_state.image_timestamps[2])[1]);
//      ImmVision::ImageDisplay("Pig", app_state.image_data.at(app_state.image_timestamps[3])[2]);
    }
    ImGui::End();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w,
                 clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Update and Render additional Platform Windows
    // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
    //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow *backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  NFD_Quit();

  glfwDestroyWindow(window);
  glfwTerminate();
}
