#include "gui/imports.h"
#include "libcbdetect/config.h"

void draw_checkerboard_config(cbdetect::Params &params) {
  if (!ImGui::Begin("Calibration Settings")) {
    ImGui::End();
    return;
  }

  ImGui::Checkbox("Show processing", &params.show_processing);


  ImGui::End();
}