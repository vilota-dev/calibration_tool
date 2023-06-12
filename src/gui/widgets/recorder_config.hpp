#pragma once

#include "gui/imports.h"

void draw_recorder_config(std::shared_ptr<vk::RosbagDatasetRecorder> &dataset_recorder,
                          std::shared_ptr<vk::CameraParams>& recorder_params,
                          std::shared_ptr<std::vector<cv::Mat>> &display_imgs) {
    if (!ImGui::Begin("Calibration Dataset Recorder Configuration")) {
        ImGui::End();
        return;
    }

    static vk::RecordMode selectedMode = vk::RecordMode::CONTINUOUS;
    static std::map<vk::RecordMode, std::string> recordModeNames = {
            {vk::RecordMode::CONTINUOUS, "Continuous"},
            {vk::RecordMode::SNAPSHOTS, "Snapshot"}
    };

    std::string selectedModeName = recordModeNames[selectedMode];
    if (ImGui::BeginCombo("Record Mode", selectedModeName.c_str())) {
        for (auto &item : recordModeNames) {
            bool isSelected = (selectedModeName == item.second);
            if (ImGui::Selectable(item.second.c_str(), isSelected)) {
                selectedMode = item.first;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::Button("Start Recording")) {
        dataset_recorder->init(*recorder_params, selectedMode, display_imgs);
        dataset_recorder->start_record();
    }

    ImGui::SameLine();

    if (ImGui::Button("Stop Recording")) {
        dataset_recorder->stop_record();
    }
    ImGui::Text("Messages recorded: %i", dataset_recorder->get_num_snapshots());

    if (dataset_recorder->is_running() && dataset_recorder->get_mode() == vk::RecordMode::SNAPSHOTS) {
        if (ImGui::Button("Take snapshot") || ImGui::IsKeyPressed(ImGuiKey_Space)) {
            dataset_recorder->take_snapshot();
        }
    }

    ImGui::End();
}