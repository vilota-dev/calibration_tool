#pragma once

#include "ui/views/view_corner_detector.hpp"
#include "ui/views/view_recorder.hpp"

#include <glad/glad.h> // Initialize with gladLoadGL()
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <string>

namespace vk {
    class Window {
    public:
        Window();
        ~Window();

        void loop();

    private:
        void frame_begin();
        void frame();
        void frame_end();

        void init_glfw();
        void init_imgui();
        void exit_glfw();
        void exit_imgui();

        GLFWwindow* window;
        std::string window_title;
        const char* glsl_version;
        ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

        /*
         * TODO: Move to ContentRegistry and call get_entries() to get the list of views to render.
         * Hardcode views for now
         * */
        ViewCornerDetector view_corner_detector;
        ViewRecorder view_recorder;
    };
} // namespace vk