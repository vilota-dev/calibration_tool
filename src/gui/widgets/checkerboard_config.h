#include "gui/imports.h"
#include "libcbdetect/config.h"

void draw_checkerboard_config(AppState &app_state, std::shared_ptr<cbdetect::Params> &params) {
    if (!ImGui::Begin("Calibration Settings")) {
        ImGui::End();
        return;
    }

    static std::map<CalibType, std::string> calibTypeNames = {
            {CalibType::AprilGrid, "AprilGrid"},
            {CalibType::Checkerboard_CBDETECT, "libcbdetect Checkerboard"},
            {CalibType::Checkerboard_OpenCV, "OpenCV Checkerboard"}};

    std::string selectedCalibTypeName = calibTypeNames[app_state.selectedCalibType];
    if (ImGui::BeginCombo("Calibration Type", selectedCalibTypeName.c_str())) {
        for (auto &item: calibTypeNames) {
            bool isSelected = (selectedCalibTypeName == item.second);
            if (ImGui::Selectable(item.second.c_str(), isSelected)) {
                app_state.selectedCalibType = item.first;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    if (app_state.selectedCalibType == CalibType::AprilGrid) {
        if (app_state.aprilgrid_files.size() == 0) {
            ImGui::Text("No AprilGrid files loaded");
        } else {
            const char *currentSelection = app_state.aprilgrid_files[app_state.selectedAprilGrid]->get_name().c_str();
            if (ImGui::BeginCombo("April Grid", currentSelection)) {
                for (int i = 0; i < app_state.aprilgrid_files.size(); i++) {
                    bool isSelected = (currentSelection == app_state.aprilgrid_files[i]->get_name().c_str());
                    if (ImGui::Selectable(app_state.aprilgrid_files[i]->get_name().c_str(), isSelected)) {
                        app_state.selectedAprilGrid = i;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::Text("tagRows: %i", app_state.aprilgrid_files[app_state.selectedAprilGrid]->getTagRows());
            ImGui::Text("tagCols: %i", app_state.aprilgrid_files[app_state.selectedAprilGrid]->getTagCols());
            ImGui::Text("tagSize: %f", app_state.aprilgrid_files[app_state.selectedAprilGrid]->getTagSize());
            ImGui::Text("tagSpacing: %f", app_state.aprilgrid_files[app_state.selectedAprilGrid]->getTagSpacing());
            ImGui::Text("lowID: %i", app_state.aprilgrid_files[app_state.selectedAprilGrid]->getLowId());
            ImGui::Text("tagFamily: %s", app_state.aprilgrid_files[app_state.selectedAprilGrid]->getTagFamily().c_str());
        }
    } else if (app_state.selectedCalibType == CalibType::Checkerboard_CBDETECT) {
        ImGui::Checkbox("Norm", &params->norm);
        ImGui::Checkbox("Polynomial Fit", &params->polynomial_fit);
        ImGui::InputInt("Norm Half Kernel Size", &params->norm_half_kernel_size);
        ImGui::InputDouble("init_loc_threshold", &params->init_loc_thr);
        ImGui::InputDouble("score_thr", &params->score_thr);
        ImGui::Checkbox("strict_grow", &params->strict_grow);
        ImGui::Checkbox("overlay", &params->overlay);
        ImGui::Checkbox("occlusion", &params->occlusion);
        const char *detectMethodNames[] = {"TemplateMatchFast", "TemplateMatchSlow", "HessianResponse", "LocalizedRadonTransform"};
        const char *currentDetectMethodName = detectMethodNames[params->detect_method];
        if (ImGui::BeginCombo("Detect Method", currentDetectMethodName)) {
            for (int i = 0; i < IM_ARRAYSIZE(detectMethodNames); i++) {
                bool isSelected = (currentDetectMethodName == detectMethodNames[i]);
                if (ImGui::Selectable(detectMethodNames[i], isSelected)) {
                    params->detect_method = static_cast<cbdetect::DetectMethod>(i);
                }
                if (isSelected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    } else if (app_state.selectedCalibType == CalibType::Checkerboard_OpenCV) {
        ImGui::InputInt("Rows", &app_state.opencv_checkerboard_params->width);
        ImGui::InputInt("Cols", &app_state.opencv_checkerboard_params->height);
        // Add flags later
    }

    ImGui::End();
}