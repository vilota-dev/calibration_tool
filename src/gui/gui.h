
#pragma once

#include "io/dataset_io.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "immvision.h"
#include <glad/glad.h> // Initialize with gladLoadGL()
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

#include "io/dataset_io.h"
#include "io/dataset_io_rosbag.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "spdlog/spdlog.h"
#include <nfd.h>
#include "rosbag_inspector.h"

#include <cstdio>
#include <filesystem>
#include <cstdlib>

struct Dataset {
    std::string file_path;
    basalt::DatasetIoInterfacePtr dataset_io;
    basalt::VioDatasetPtr vio_dataset;
    // Variable to store number of cameras
    size_t num_cams;
    cv::Mat image; // Now we just have to read the ManagedImage into this fella and we are done. follow line 92 of dataset_io_uzh
//    std::vector<basalt::ImageData> &img_vec; // Refers to the images for each camera at a certain timestamp

    Dataset() = default;

    void loadDataset(const std::filesystem::path &path) {
        this->dataset_io = basalt::DatasetIoFactory::getDatasetIo("bag");

        this->file_path = path.string();
        dataset_io->read(this->file_path);

        this->vio_dataset = dataset_io->get_data();

        std::vector<int64_t> new_image_timestamps;
        for (long long i : vio_dataset->get_image_timestamps()) { // Change back to non-range based for loop if cause error
            new_image_timestamps.push_back(i);
        }

        this->vio_dataset->get_image_timestamps() = new_image_timestamps;
    }

    std::vector<basalt::ImageData> feed_images(int64_t t_ns) {
      const std::vector<basalt::ImageData> &img_vec = vio_dataset->get_image_data(t_ns);

      for (size_t i = 0; i < img_vec.size(); i++) {
        // Iterate over each ImageData in the vector
        for (const auto& image_data : img_vec) {
          // Extract the ManagedImage<uint16_t> pointer from the ImageData
          const basalt::ManagedImage<uint16_t>::Ptr& managed_image_ptr = image_data.img;

          // Check if the managed_image_ptr is valid
          if (managed_image_ptr) {
            // Extract the image properties from the managed_image_ptr
            size_t width = managed_image_ptr->w;
            size_t height = managed_image_ptr->h;
            size_t pitch_bytes = managed_image_ptr->pitch;

            // Create a cv::Mat with the appropriate size and data type
            cv::Mat image_mat_16u(height, width, CV_16U, managed_image_ptr->ptr, pitch_bytes);

            // Create an 8-bit version of the image by right shifting the 16-bit values
            cv::Mat image_mat_8u(height, width, CV_8U);
            for (int y = 0; y < height; ++y) {
              const uint16_t* src_ptr = image_mat_16u.ptr<uint16_t>(y);
              uchar* dst_ptr = image_mat_8u.ptr<uchar>(y);
              for (int x = 0; x < width; ++x) {
                dst_ptr[x] = static_cast<uchar>(src_ptr[x] >> 8);  // Right shift by 8 bits
              }
            }

            // Do something with the converted cv::Mat (e.g., perform image processing or display)
            // ...

            // Note: The cv::Mat `image_mat_8u` contains the scaled-down 8-bit values.
            // Make sure the ManagedImage remains valid during the lifetime of `image_mat_8u`.
            // otherwise, the cv::Mat will contain invalid data.
          }
        }
      }

      spdlog::info("Loaded dataset with {} images", img_vec.size());
      return img_vec;
    }
};

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

void draw_main_menu_bar(Dataset &data) {
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

    Dataset big_data;
    big_data.loadDataset("/Users/tejas/Developer/vilota-dev/calibration_tool/data/2023-04-21/run1/2023-04-21-vk180-calib-run1.bag");

    auto &timestamps = big_data.vio_dataset->get_image_timestamps();
    std::vector<cv::Mat> display_imgs;
    for (const auto& timestamp : timestamps) {
      const std::vector<basalt::ImageData> &img_vec = big_data.vio_dataset->get_image_data(timestamp);
//      spdlog::info("Timestamp: {} | Loaded {} images", timestamp, img_vec.size()); // Prints correct size
//
//      for (const auto& image_data : img_vec) {
//        spdlog::info("Exposure is {}", image_data.exposure);
//      }

      for (size_t i = 0; i < img_vec.size(); i++) {
        // Iterate over each ImageData in the vector
        for (const auto& image_data : img_vec) {
          // Extract the ManagedImage<uint16_t> pointer from the ImageData
          const basalt::ManagedImage<uint16_t>::Ptr& managed_image_ptr = image_data.img;

          // Check if the managed_image_ptr is valid
          if (managed_image_ptr) {
            spdlog::info("SUCEESS ITCH");
            // Extract the image properties from the managed_image_ptr
            size_t width = managed_image_ptr->w;
            size_t height = managed_image_ptr->h;
            size_t pitch_bytes = managed_image_ptr->pitch;

            // Create a cv::Mat with the appropriate size and data type
            cv::Mat image_mat_16u(height, width, CV_16U, managed_image_ptr->ptr, pitch_bytes);

            // Create an 8-bit version of the image by right shifting the 16-bit values
            cv::Mat image_mat_8u(height, width, CV_8U);
            for (int y = 0; y < height; ++y) {
              const uint16_t* src_ptr = image_mat_16u.ptr<uint16_t>(y);
              uchar* dst_ptr = image_mat_8u.ptr<uchar>(y);
              for (int x = 0; x < width; ++x) {
                dst_ptr[x] = static_cast<uchar>(src_ptr[x] >> 8);  // Right shift by 8 bits
              }
            }

            display_imgs.push_back(image_mat_8u);

            // Do something with the converted cv::Mat (e.g., perform image processing or display)
            // ...

            // Note: The cv::Mat `image_mat_8u` contains the scaled-down 8-bit values.
            // Make sure the ManagedImage remains valid during the lifetime of `image_mat_8u`.
            // otherwise, the cv::Mat will contain invalid data.
          }
        }
      }
    }



    spdlog::info("Loaded {} images", display_imgs.size()); // Does not print correct size.

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
        std::cout << "Failed to initialize GLAD" << std::endl;
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

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    // - Our Emscripten build process allows embedding fonts to be accessible at runtime from the "fonts/" folder. See Makefile.emscripten for details.
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        config_gui();
        calibrate_gui();

        draw_main_menu_bar(big_data);

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
              spdlog::info("Image vector size: {}", display_imgs.size());
//              spdlog::
//              images = big_data.feed_images();
//              ImmVision::Image("Original", show_this[0], &dogState.immvisionParams);
            }
            ImmVision::ImageDisplay("Cat", display_imgs[0]);

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
