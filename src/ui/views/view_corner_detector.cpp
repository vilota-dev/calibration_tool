#include "ui/views/view_corner_detector.hpp"

#include "app_state.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>
#include <spdlog/spdlog.h>
#include "nfd.hpp"

#include <unistd.h>

ViewCornerDetector::ViewCornerDetector()
    : View("Corner Detector"),
        show_corners(true), show_corners_rejected(false), selected_rosbag(0), selected_frame(0), selected_aprilgrid(0),
        image_params(ImmVision::ImageParams()), detection_type(DetectionType::Checkerboard),
        cb_width(8), cb_height(6), cb_row_spacing(0.04f), cb_col_spacing(0.04f),
        adaptive_thresh(true), normalize_image(true), filter_quads(true), fast_check(true), enable_subpix_refine(true),
        cam_types(std::vector<std::string>({"kb4", "kb4"})){
    this->image_params.RefreshImage = true;
}

void ViewCornerDetector::draw_config() {
    ImGui::BeginChild("Corner Detector Config", ImVec2(0, 50), true);

    auto &app_state = AppState::get_instance();

    if (ImGui::BeginCombo("ROS .bag Datasets",
                          app_state.rosbag_files[this->selected_rosbag]->get_file_path().c_str())) {
        for (int i = 0; i < app_state.rosbag_files.size(); i++) {
            bool is_selected = (this->selected_rosbag == i);
            if (ImGui::Selectable(app_state.rosbag_files[i]->get_file_path().c_str(), is_selected)) {
                this->selected_rosbag = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Checkbox("Show corners", &this->show_corners); ImGui::SameLine();
    ImGui::Checkbox("Show corners rejected", &this->show_corners_rejected); ImGui::SameLine();

    if (ImGui::Button("Detect Corners")) {
        ImGui::OpenPopup("Detection Config");
    }

    // TODO: Disable this if detect corners has not been pressed yet.
    if (ImGui::Button("Launch vk_calibrate")) {
        ImGui::OpenPopup("vk_calibrate Config");
    }

    // Open the popup if the button is clicked.
    this->draw_detection_popup();
    this->draw_vkcalibrate_popup();

    ImGui::EndChild();
}

void ViewCornerDetector::draw_vkcalibrate_popup() {
    if (ImGui::BeginPopupModal("vk_calibrate Config", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {

        // TODO: Add better enum for different dataset type
        auto &app_state = AppState::get_instance();
        std::string dataset_path = app_state.rosbag_files[this->selected_rosbag]->get_file_path();

        // Open NFD for the folder selection for the result path
        static std::string result_path;
        ImGui::InputText("Result Path", &result_path); ImGui::SameLine();
        if (ImGui::Button("Select Result Path")) {
            NFD::Guard nfdGuard;
            NFD::UniquePath outPath;
            nfdresult_t result = NFD::PickFolder(outPath);
            if (result == NFD_OKAY) {
                result_path = outPath.get();
                spdlog::info("NFD: Picked folder {}", result_path);
            } else if (result == NFD_CANCEL) {
                spdlog::info("NFD: User pressed cancel.");
            } else {
                spdlog::error("NFD: Error: {}", NFD::GetError());
            }
        }

        // User can either choose priors that are preloaded or add their own types
        static int use_prior = 0;
        ImGui::RadioButton("Preloaded Priors", &use_prior, 0); ImGui::SameLine();
        ImGui::RadioButton("Custom", &use_prior, 1);

        if (use_prior == 0) {
            ImGui::Text("NOT IMPLEMENTED FULLY YET, Please use the custom option");
            // Load the json files from /opt/vilota/bin directory
            /*
             * This is the proper code
             * */
//            static std::vector<std::string> priors;
//            static int selected_prior = 0;
//            // use the get_json_files to load the json files from teh /opt/vilota/bin directory
//            std::string priors_dir = "/opt/vilota/bin/priors/";
//            std::vector<std::string> json_files = get_json_files(priors_dir);
//
//            if (ImGui::BeginCombo("Priors", json_files[selected_prior].c_str())) {
//                for (int i = 0; i < json_files.size(); i++) {
//                    bool is_selected = (selected_prior == i);
//                    if (ImGui::Selectable(json_files[i].c_str(), is_selected)) {
//                        selected_prior = i;
//                    }
//                    if (is_selected) {
//                        ImGui::SetItemDefaultFocus();
//                    }
//                }
//                ImGui::EndCombo();
//            }

            /*
             * TODO: This is temporary hacked together code for user to test
             * Only allows two hardcoded priors, under directory opt/vilota/bin/priors/kb4 and radtan
             * */
            static std::vector<std::string> priors = {"kb4", "radtan"};
            static int selected_prior = 0;
            if (ImGui::BeginCombo("Priors", priors[selected_prior].c_str())) {
                for (int i = 0; i < priors.size(); i++) {
                    bool is_selected = (selected_prior == i);
                    if (ImGui::Selectable(priors[i].c_str(), is_selected)) {
                        selected_prior = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

        } else {
            /*
             * Allow user to add their own camera types and number of them.
             * TODO: Change items to support all the models vk_calibrate supports
             * */
            const char* items[] = {"pinhole-radtan8", "kb4"};
            static int item_current = 0;
            ImGui::Combo("Camera Type", &item_current, items, IM_ARRAYSIZE(items));
            ImGui::SameLine();
            if (ImGui::Button("Add Camera Type")) {
                this->cam_types.emplace_back(items[item_current]);
            }

            for (int i = 0; i < cam_types.size(); ++i) {
                ImGui::Text("%s", cam_types[i].c_str());

                ImGui::SameLine();
                if (this->cam_types.size() > 1) { // Don't allow zero sized
                    if (ImGui::SmallButton("x")) {
                        // Remove the item from the list
                        cam_types.erase(cam_types.begin() + i);
                        --i;
                    }
                }
            }
        }

        // Open NFD for checkerboard json file
        // TODO: Remove this once we serialize into the cereal dump
        static std::string cb_path;
        ImGui::InputText("Checkerboard .json Path", &cb_path); ImGui::SameLine();
        if (ImGui::Button("Select cb .json Path")) {
            NFD::Guard nfdGuard;
            NFD::UniquePath outPath;
            nfdfilteritem_t bagFilter[1] = {{"Checkerboard .json file", "json"}}; // support for png later
            nfdresult_t result = NFD::OpenDialog(outPath, bagFilter, 1);

            if (result == NFD_OKAY) {
                const std::string path = outPath.get();
                cb_path = outPath.get();
            } else if (result == NFD_CANCEL) {
                spdlog::debug("User pressed cancel.");
            } else {
                spdlog::error("File upload failed. Error: {}", NFD_GetError());
            }
        }

        if (ImGui::Button("Launch vk_calibrate", ImVec2(120, 0))) {
            this->launch_vkcalibrate(dataset_path, cb_path, result_path, this->cam_types);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();

        if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }

        ImGui::EndPopup();
    }
}
void ViewCornerDetector::draw_cam_view() {
    ImGui::BeginChild("Camera Views", ImVec2(0, 0), true);

    auto &app_state = AppState::get_instance();

    // Get the selected rosbag
    auto &rosbag = app_state.rosbag_files[this->selected_rosbag];

    // Draw the image slider and ImmVision
    ImGui::SliderInt("Selected Frame", &selected_frame, 0, rosbag->get_image_timestamps().size() - 1);
    ImGui::NewLine();

    auto ts = rosbag->get_image_timestamps()[this->selected_frame];
    auto img_to_display = rosbag->image_data.at(ts);

    auto num_cams = static_cast<int>(rosbag->get_num_cams());

    ImGui::Columns(num_cams);

    int idx = 0;
    for (auto &cam_name : rosbag->get_camera_names()) {
        // Quick hack to ensure each immvision object has a different zoom key without
        // creating multiple instances of the ImageParams object (HACK DOESN'T WORK)
        this->image_params.ZoomKey = cam_name;
        ImmVision::Image(cam_name, img_to_display[idx], &this->image_params);
        ImGui::NextColumn();
        idx++;
    }

    ImGui::EndChild();
}

void ViewCornerDetector::draw_content() {
    if (!ImGui::Begin(this->get_name().c_str())) {
        ImGui::End();
        return;
    }

    auto &app_state = AppState::get_instance();

    /*
     * If there's no ROS .bag loaded, then we hide everything.
     * */
    if (app_state.rosbag_files.size() == 0) {
        // Note that 0 tells ImGui to "just use the default"
        ImVec2 button_size = ImGui::CalcItemSize(ImVec2{300, 50}, 0.0f, 0.0f);

        ImVec2 avail = ImGui::GetWindowSize();

        // Calculate center of window for button.
        ImVec2 centre_position_for_button{
            // we have two widgets, so twice the size - and we need to account for the spacing in the middle
                (avail.x - button_size.x) / 2,
                (avail.y - button_size.y) / 2
        };

        // tell ImGui to render the button at the new position
        ImGui::SetCursorPos(centre_position_for_button);

        if (ImGui::Button("Load ROS .bag dataset to continue", button_size)) {
            app_state.load_dataset();
        }
    } else {
        this->draw_config();
        this->draw_cam_view();
    }

    ImGui::End();
}

void ViewCornerDetector::draw_detection_popup() {
    auto &app_state = AppState::get_instance();

    if (ImGui::BeginPopupModal("Detection Config", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (ImGui::BeginCombo("corner_detection_type", this->detection_type._to_string())) {
            for (DetectionType type: DetectionType::_values()) {
                bool is_selected = (detection_type == type);
                if (ImGui::Selectable(type._to_string(), is_selected)) {
                    detection_type = type;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        /*
         * Draw calibration board specific details.
         * libcbdetect is removed for now, since we are not using it. Refer to previous commits if you want to use it.
         * */
        switch (detection_type) {
            case DetectionType::AprilGrid: {
                if (ImGui::Button("Load AprilGrid")) {
                    app_state.load_aprilgrid();
                }
                if (app_state.aprilgrid_files.size() != 0) {
                    ImGui::SameLine();
                    const char *currentSelection = app_state.aprilgrid_files[this->selected_aprilgrid]->get_name().c_str();

                    if (ImGui::BeginCombo("April Grid", currentSelection)) {
                        for (int i = 0; i < app_state.aprilgrid_files.size(); i++) {
                            bool isSelected = (currentSelection == app_state.aprilgrid_files[i]->get_name().c_str());
                            if (ImGui::Selectable(app_state.aprilgrid_files[i]->get_name().c_str(), isSelected)) {
                                this->selected_aprilgrid = i;
                            }
                            if (isSelected) {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }

                    ImGui::Text("tagRows: %i", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagRows());
                    ImGui::Text("tagCols: %i", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagCols());
                    ImGui::Text("tagSize: %f", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagSize());
                    ImGui::Text("tagSpacing: %f", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagSpacing());
                    ImGui::Text("lowID: %i", app_state.aprilgrid_files[this->selected_aprilgrid]->getLowId());
                    ImGui::Text("tagFamily: %s", app_state.aprilgrid_files[this->selected_aprilgrid]->getTagFamily().c_str());
                }
                break;
            }
            case DetectionType::Checkerboard: {
                ImGui::InputInt("Rows", &this->cb_width);
                ImGui::InputInt("Cols", &this->cb_height);
                ImGui::InputFloat("Row Spacing (in meters)", &this->cb_row_spacing);
                ImGui::InputFloat("Column Spacing (in meters)", &this->cb_col_spacing);

                ImGui::Checkbox("Adaptive Threshold", &this->adaptive_thresh);
                ImGui::Checkbox("Normalize Image", &this->normalize_image);
                ImGui::Checkbox("Filter Quads", &this->filter_quads);
                ImGui::Checkbox("Fast Check", &this->fast_check);
                ImGui::Checkbox("Enable sub-pixel refinement",
                                &this->enable_subpix_refine);
                break;
            }
        }

        if (ImGui::Button("Detect!", ImVec2(120, 0))) {
            this->detect_corners();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();

        if (ImGui::Button("Cancel", ImVec2(120, 0))) { ImGui::CloseCurrentPopup(); }

        ImGui::EndPopup();
    }
}

void ViewCornerDetector::detect_corners() {
    spdlog::debug("Detecting corners with {} mode", detection_type._to_string());

    switch (this->detection_type) {
        case DetectionType::AprilGrid: {
            //NOLINTNEXTLINE
            AppState::get_instance().submit_task([this]() {
                auto &app_state = AppState::get_instance();
                auto params = std::make_shared<basalt::AprilGridParams>(
                        app_state.aprilgrid_files[this->selected_aprilgrid]);
                auto calibrator = std::make_unique<basalt::Calibrator>(
                        app_state.rosbag_files[this->selected_rosbag]);

                calibrator->detectCorners(params);
                this->draw_corners();
            });
            break;
        }
        case DetectionType::Checkerboard: {
            //NOLINTNEXTLINE
            AppState::get_instance().submit_task([this]() {
                auto &app_state = AppState::get_instance();
                auto params = std::make_shared<basalt::OpenCVCheckerboardParams>(
                        this->cb_width, this->cb_height, this->adaptive_thresh, this->normalize_image,
                        this->filter_quads, this->fast_check, this->enable_subpix_refine);
                auto calibrator = std::make_unique<basalt::Calibrator>(
                        app_state.rosbag_files[this->selected_rosbag]);

                calibrator->detectCorners(params);
                this->draw_corners();
            });
            break;
        }
    }

}

void ViewCornerDetector::draw_corners() {
    //NOLINTNEXTLINE
    AppState::get_instance().submit_task([this]() {
        spdlog::trace("Started drawing corners");
        auto &app_state = AppState::get_instance();

        for (auto ts: app_state.rosbag_files[this->selected_rosbag]->get_image_timestamps()) {
            spdlog::trace("Drawing corners for timestamp: {}", ts);
            std::vector<cv::Mat> &img_vec = app_state.rosbag_files[this->selected_rosbag]->image_data.at(ts);

            for (int cam_num = 0; cam_num < app_state.rosbag_files[this->selected_rosbag]->get_num_cams(); cam_num++) {
                auto tcid = basalt::TimeCamId(ts, cam_num);
                const basalt::CalibCornerData &cr = app_state.rosbag_files[this->selected_rosbag]->calib_corners.at(tcid);
//          const CalibCornerData &cr_rej = this->rosbag_files[this->selected]->calib_corners_rejected.at(tcid);

                for (size_t i = 0; i < cr.corners.size(); i++) {
                    // The radius is the threshold used for maximum displacement. The search region is slightly larger.
                    const float radius = static_cast<float>(cr.radii[i]);
                    const Eigen::Vector2d &c = cr.corners[i];
                    // Convert to cv::Point2f for opencv drawing
                    std::vector<cv::Point2f> cv_corners;
                    for (const auto &corner : cr.corners) {
                        cv_corners.emplace_back(corner[0], corner[1]);
                    }
                    const auto idx = cr.corner_ids[i];

                    switch (this->detection_type) {
                        case DetectionType::AprilGrid: {
                            cv::circle(img_vec[cam_num], cv::Point2d(c[0], c[1]), static_cast<int>(radius),
                                       cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

                            cv::putText(img_vec[cam_num], std::to_string(idx), cv::Point2i(c[0] - 12, c[1] - 6),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                            break;
                        }
                        case DetectionType::Checkerboard: {
                            cv::Size size(this->cb_width, this->cb_height);
                            cv::drawChessboardCorners(img_vec[cam_num], size, cv::Mat(cv_corners), true);
                            break;
                        }
                    }
                }

            }
        }
        spdlog::trace("Finished drawing corners");
    });
}

void ViewCornerDetector::launch_vkcalibrate(std::string dataset_path, std::string cb_path,
                                            std::string result_path, std::vector<std::string> cam_types) {
    // Construct command using stringstream
    std::stringstream command_stream;
    command_stream << "/opt/vilota/bin/vk_calibrate";
    command_stream << " --dataset-path " << dataset_path;
    command_stream << " --dataset-type bag";
    command_stream << " --checkerboard " << cb_path;
    command_stream << " --result-path " << result_path;
    command_stream << " --cam-types";
    for (const auto &cam_type : cam_types) {
        command_stream << " " << cam_type;
    }
    std::string command = command_stream.str();

    std::thread t([command, dataset_path] {
        spdlog::info("Running command on vk_calibrate_thread: {}", command.c_str());
        int success = std::system(command.c_str());
        spdlog::info("vk_calibrate launched with exit code: {}", success);
    });

    t.detach();
}