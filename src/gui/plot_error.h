#include "imgui.h"
#include "implot.h"

void plot_mean_error(std::vector<float> &frame_rates) {
  frame_rates.erase(frame_rates.begin());
  frame_rates.push_back(1.f / ImGui::GetIO().DeltaTime);

  if (!ImGui::Begin("Long running task")) {
    // Early out if the window is collapsed, as an optimization.
    ImGui::End();
    return;
  }

  if (ImPlot::BeginSubplots("", 1, 1, ImVec2(1200, 800))) {
    if (ImPlot::BeginPlot("")) {
      ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 5.f);

      ImPlot::SetupAxis(ImAxis_X1, "Frame Number", ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxis(ImAxis_Y1, "Frame Rate");
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 100);

      std::vector<float> local_samples(frame_rates.size());
      std::generate(local_samples.begin(), local_samples.end(), [n = 0]() mutable { return 1.0 * n++; });
      ImPlot::PlotLine("Frame Rate", local_samples.data(), frame_rates.data(), frame_rates.size());
      ImPlot::EndPlot();

      ImPlot::PopStyleVar(ImPlotStyleVar_LineWeight);
    }
    ImPlot::EndSubplots();
  }

  ImGui::End();
}