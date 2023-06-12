#pragma once

#include "io/dataset_io.h"
#include "gui/widgets/rosbag_inspector.h"
#include "gui/widgets/plot_error.h"
#include "gui/widgets/img_display.h"
#include "gui/widgets/checkerboard_config.h"
#include "gui/widgets/recorder_config.hpp"
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>

#include "spdlog/spdlog.h"
#include <nfd.hpp>

#include <cstdio>
#include <filesystem>
#include <cstdlib>

#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif

void config_gui(AppState &app_state) {
  ImGui::Begin("Calibration GUI");

  static bool show_corners = false;
  ImGui::Checkbox("Show corners", &show_corners);

  static bool show_corners_rejected = false;
  ImGui::Checkbox("Show corners rejected", &show_corners_rejected);

  // make a slider int
  static int threshold = 0;
  ImGui::SliderInt("Gaussian Blur", &threshold, 0, 101);

  if (ImGui::Button("Load Dataset")) {
    spdlog::info("Loading dataset");
    // Run the CalibHelper::
  }

  if (ImGui::Button("Detect Corners")) {
    spdlog::trace("Starting corner detection");
    app_state.detectCorners();
  }

  if (ImGui::Button("Detect Checkerboard Corners")) {
    spdlog::trace("Starting checkerboard corner detection");
    app_state.detectCheckerboardCorners();
  }

  if (ImGui::Button("Draw Checkerboard")) {
    spdlog::trace("Starting checkerboard drawing");
    app_state.drawCheckerboardCorners();
  }

  if (ImGui::Button("Draw Corners")) {
    spdlog::trace("Starting corner drawing");
    app_state.drawCorners();
  }

  if (ImGui::Button("Optimize")) {
    spdlog::info("Starting calibration");
    // Optimize function
  }
  ImGui::End();
}

void draw_main_menu_bar(AppState &app_state) {
  ImGui::BeginMainMenuBar();
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Load ROS .bag file")) {
      NFD::Guard nfdGuard;
      NFD::UniquePath outPath;
      nfdfilteritem_t bagFilter[1] = {{"ROS .bag file", "bag"}}; // support for png later
      nfdresult_t result = NFD::OpenDialog(outPath, bagFilter, 1);
      if (result == NFD_OKAY) {
        app_state.loadDataset(outPath.get());
        spdlog::debug("Success! File loaded from {}", outPath.get());
      } else if (result == NFD_CANCEL) {
        spdlog::debug("User pressed cancel.");
      } else {
        spdlog::error("Error: {}", NFD_GetError());
      }
    }

    if (ImGui::MenuItem("Load AprilGrid .json file")) {
      NFD::Guard nfdGuard;
      NFD::UniquePath outPath;
      static nfdfilteritem_t aprilgridFilter[1] = {{"AprilGrid .json file", "json"}}; // support for png later
      nfdresult_t result = NFD::OpenDialog(outPath, aprilgridFilter, 1);
      if (result == NFD_OKAY) {
        app_state.loadDataset(outPath.get());
        spdlog::debug("Success! File loaded from {}", outPath.get());
      } else if (result == NFD_CANCEL) {
        spdlog::debug("User pressed cancel.");
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
  AppState app_state;

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
  // https://github.com/IntelRealSense/librealsense/blob/b874e42685aed1269bc57a2fe5bf14946deb6ede/tools/rosbag-inspector/rs-rosbag-inspector.cpp#L89
  // for drag n drop upload files

  // Load Glad
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    spdlog::error("Failed to initialize GLAD");
    std::exit(EXIT_FAILURE);
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
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
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()); // Make main window into dockspace

    draw_main_menu_bar(app_state);
//    plot_mean_error(app_state.frame_rates);
    config_gui(app_state);
    draw_rosbag_inspector(app_state);
    draw_checkerboard_config(app_state, app_state.checkerboard_params);
    draw_recorder_config(app_state.dataset_recorder, app_state.recorder_params);
    if (app_state.rosbag_files.size() > 0) {
//      draw_calibration_settings(app_state);
      img_display(app_state.immvisionParams, app_state);
    }

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
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}
