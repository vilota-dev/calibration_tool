#include "gui/imports.h"
#include "libcbdetect/config.h"

void draw_checkerboard_config(AppState &app_state, std::shared_ptr<cbdetect::Params> &params) {
  if (!ImGui::Begin("Calibration Settings")) {
    ImGui::End();
    return;
  }

  static std::map<CalibType, std::string> calibTypeNames = {
    {CalibType::AprilGrid, "AprilGrid"},
    {CalibType::Checkerboard, "Checkerboard"}
  };

  std::string selectedCalibTypeName = calibTypeNames[app_state.selectedCalibType];
  if (ImGui::BeginCombo("Calibration Type", selectedCalibTypeName.c_str())) {
    for (auto &item : calibTypeNames) {
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

  ImGui::Checkbox("show_debug_image", &params->show_debug_image);
  ImGui::Checkbox("Show processing", &params->show_processing);
  ImGui::Checkbox("Norm", &params->norm);
  ImGui::Checkbox("Polynomial Fit", &params->polynomial_fit);
  ImGui::InputInt("Norm Half Kernel Size", &params->norm_half_kernel_size);
  ImGui::InputDouble("init_loc_threshold", &params->init_loc_thr);
  ImGui::InputDouble("score_thr", &params->score_thr);
  ImGui::Checkbox("strict_grow", &params->strict_grow);
  ImGui::Checkbox("overlay", &params->overlay);
  ImGui::Checkbox("occlusion", &params->occlusion);
  const char* detectMethodNames[] = { "TemplateMatchFast", "TemplateMatchSlow", "HessianResponse", "LocalizedRadonTransform" };
  const char* currentDetectMethodName = detectMethodNames[params->detect_method];
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


  ImGui::End();
}