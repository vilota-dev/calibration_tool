#pragma once

#include "gui/imports.h"

void draw_recorder_config(std::shared_ptr<vk::RosbagDatasetRecorder> &dataset_recorder,
                          std::shared_ptr<vk::CameraParams>& recorder_params,
                          std::shared_ptr<std::vector<cv::Mat>> &display_imgs) {
    if (!ImGui::Begin("Recorder Config")) {
        ImGui::End();
        return;
    }

    static vk::RecordMode selectedMode = vk::RecordMode::SNAPSHOTS;
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

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.2f, 0.4f, 0.2f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.2f, 0.6f, 0.2f, 1.0f});
    if (ImGui::Button("Start Recording")) {
        dataset_recorder->init(*recorder_params, selectedMode, display_imgs);
        dataset_recorder->start_record();
    }
    ImGui::PopStyleColor(2);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.4f, 0.2f, 0.2f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.6f, 0.2f, 0.2f, 1.0f});
    if (ImGui::Button("Stop Recording")) {
        dataset_recorder->stop_record();
    }
    ImGui::PopStyleColor(2);

    ImGui::Text("Messages recorded: %i", dataset_recorder->get_num_snapshots());

    if (dataset_recorder->is_running() && dataset_recorder->get_mode() == vk::RecordMode::SNAPSHOTS) {
        if (ImGui::Button("Take snapshot") || ImGui::IsKeyPressed(ImGuiKey_Space)) {
            dataset_recorder->take_snapshot();
        }
    }

    ImGui::End();
}