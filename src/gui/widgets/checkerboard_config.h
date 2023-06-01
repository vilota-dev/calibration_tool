#include "gui/imports.h"
#include "libcbdetect/config.h"

void draw_checkerboard_config(cbdetect::Params &params) {
  if (!ImGui::Begin("Calibration Settings")) {
    ImGui::End();
    return;
  }

  ImGui::Checkbox("show_debug_image", &params.show_debug_image);
  ImGui::Checkbox("Show processing", &params.show_processing);
  ImGui::Checkbox("Norm", &params.norm);
  ImGui::Checkbox("Polynomial Fit", &params.polynomial_fit);
  ImGui::InputInt("Norm Half Kernel Size", &params.norm_half_kernel_size);
  ImGui::InputDouble("init_loc_threshold", &params.init_loc_thr);
  ImGui::InputDouble("score_thr", &params.score_thr);
  ImGui::Checkbox("strict_grow", &params.strict_grow);
  ImGui::Checkbox("overlay", &params.overlay);
  ImGui::Checkbox("occlusion", &params.occlusion);
  const char* detectMethodNames[] = { "TemplateMatchFast", "TemplateMatchSlow", "HessianResponse", "LocalizedRadonTransform" };
  const char* currentDetectMethodName = detectMethodNames[params.detect_method];
  if (ImGui::BeginCombo("Detect Method", currentDetectMethodName)) {
    for (int i = 0; i < IM_ARRAYSIZE(detectMethodNames); i++) {
      bool isSelected = (currentDetectMethodName == detectMethodNames[i]);
      if (ImGui::Selectable(detectMethodNames[i], isSelected)) {
        params.detect_method = static_cast<cbdetect::DetectMethod>(i);
      }
      if (isSelected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }

  // Enum combobox for detect_method parameter in params
//  static int detect_method_current_idx = 0;
//  ImGui::Combo("Detect Method", &detect_method_current_idx, detect_method_items, IM_ARRAYSIZE(detect_method_items));
//  if (ImGui::BeginCombo("Detect Method", detect_method_items[detect_method_current_idx])) {
//    for (int n = 0; n < IM_ARRAYSIZE(detect_method_items); n++) {
//      const bool is_selected = (detect_method_current_idx == n);
//      if (ImGui::Selectable(detect_method_items[n], is_selected)) {
//        detect_method_current_idx = n;
//        params.detect_method = static_cast<cbdetect::DetectMethod>(n);
//      }
//      if (is_selected) {
//        ImGui::SetItemDefaultFocus();
//      }
//    }
//    ImGui::EndCombo();
//  }


  ImGui::End();
}