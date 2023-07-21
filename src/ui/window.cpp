#include "ui/window.hpp"

#include "app_state.hpp"

#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>
#include <implot.h>
#include <spdlog/spdlog.h>

namespace vk {
    Window::Window() {
        this->init_glfw();
        this->init_imgui();

    }

    Window::~Window() {
        this->exit_imgui();
        this->exit_glfw();
    }

    void Window::loop() {
        while (!glfwWindowShouldClose(this->window)) {
            glfwPollEvents();

            this->frame_begin();
            this->frame();
            this->frame_end();

            glfwSwapInterval(0);
        }
    }

    void Window::frame_begin() {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()); // Make main window into dockspace

        // Render Main Menu
        if (ImGui::BeginMainMenuBar()) {
            // Make color dark grey
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.2f, 0.2f, 0.2f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.3f, 0.3f, 0.3f, 1.0f});
            if (ImGui::SmallButton("Load ROS .bag file")) {
                AppState::get_instance().load_dataset();
            }

            ImGui::Separator();

            ImGui::PopStyleColor(2);
            ImGui::EndMainMenuBar();
        }
    }

    void Window::frame() {
        // Render views
        // TODO: Move to ContentRegistry and call get_entries() to get the list of views to render in a loop
        this->view_corner_detector.draw_content();
#ifndef HUAWEI
        this->view_recorder.draw_content();
        this->view_rosbag_inspector.draw_content();
#endif
    }

    void Window::frame_end() {
        // Render UI
        ImGui::Render();

        int display_width, display_height;
        glfwGetFramebufferSize(window, &display_width, &display_height);
        glViewport(0, 0, display_width, display_height);
        glClearColor(0.00F, 0.00F, 0.00F, 0.00F);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Update and Render additional Platform Windows
        // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
        ImGuiIO &io = ImGui::GetIO();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow *backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }

    void Window::init_glfw() {
        // Setup GLFW window
        glfwSetErrorCallback([](int error, const char *description) {
            spdlog::error("GLFW Error {}: {}", error, description);
        });

        if (!glfwInit()) {
            spdlog::error("Failed to initialize GLFW");
            std::exit(EXIT_FAILURE);
        }

        // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100
        this->glsl_version = "#version 100";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
        // GL 3.2 + GLSL 150
        this->glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
        // GL 3.0 + GLSL 130
        this->glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

        this->window_title = "vk Calibration Tool";
        this->window = glfwCreateWindow(1280, 720, this->window_title.c_str(), nullptr, nullptr);

        if (window == nullptr) {
            spdlog::error("Failed to create GLFW window");
            glfwTerminate();
            std::exit(1);
        }

        // https://github.com/IntelRealSense/librealsense/blob/b874e42685aed1269bc57a2fe5bf14946deb6ede/tools/rosbag-inspector/rs-rosbag-inspector.cpp#L89
        // for drag n drop upload files
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); // Enable vsync

        // Load Glad
        if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
            spdlog::error("Failed to initialize GLAD");
            std::exit(EXIT_FAILURE);
        }
    }

    void Window::init_imgui() {
        IMGUI_CHECKVERSION();

        ImGui::CreateContext();
        ImPlot::CreateContext();

        ImGuiIO &io = ImGui::GetIO();
        (void) io;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
        io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows

        ImGui::StyleColorsDark();

        // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
        ImGuiStyle &style = ImGui::GetStyle();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            style.WindowRounding = 0.0f;
            style.Colors[ImGuiCol_WindowBg].w = 1.0f;
        }

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(this->glsl_version);
    }

    void Window::exit_glfw() {
        glfwDestroyWindow(this->window);
        glfwTerminate();

        this->window = nullptr;
    }

    void Window::exit_imgui() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
    }
} // namespace vk