#pragma once

#include "gui/imports.h"

void draw_recorder_config(std::shared_ptr<vk::RosbagDatasetRecorder> &dataset_recorder,
                          std::shared_ptr<vk::CameraParams>& recorder_params,
                          std::shared_ptr<std::vector<cv::Mat>> &display_imgs) {
    if (!ImGui::Begin("Recorder Config")) {
        ImGui::End();
        return;
    }

    static int custom = 0;
    ImGui::RadioButton("Preset", &custom, 0); ImGui::SameLine();
    ImGui::RadioButton("Custom", &custom, 1);

    if (!custom) {
        std::vector<Preset>& presets = getPresets();

        static int preset_current_idx = 0;
        const char* selectedPresetName = presets[preset_current_idx].name.c_str();
        if (ImGui::BeginCombo("Choose Custom Preset", selectedPresetName)) {
            for (int n = 0; n < presets.size(); n++) {
                const bool is_selected = (preset_current_idx == n);
                if (ImGui::Selectable(presets[n].name.c_str(), is_selected)) {
                    preset_current_idx = n;
                    recorder_params->camera_topics = presets[n].topics;
                    recorder_params->tf_prefix = presets[n].tf_prefix;
                }

                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

    } else {
        // Add tf_prefix string input as well as cam_topics
        ImGui::InputText("tf_prefix", &recorder_params->tf_prefix);

        static char inputBuffer[256] = "";
        bool clicked = ImGui::InputText("New Topic", inputBuffer, IM_ARRAYSIZE(inputBuffer), ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        if (ImGui::Button("Add") || clicked) {
            recorder_params->camera_topics.push_back(std::string(inputBuffer));
            memset(inputBuffer, 0, sizeof(inputBuffer));
        }

        for (int i = 0; i < recorder_params->camera_topics.size(); ++i) {
            ImGui::Text("%s", recorder_params->camera_topics[i].c_str());

            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                // Remove the item from the list
                recorder_params->camera_topics.erase(recorder_params->camera_topics.begin() + i);
                --i;
            }
        }
    }

    ImGui::NewLine(); ImGui::Separator(); ImGui::NewLine();
    
    static vk::RecordMode selectedMode = vk::RecordMode::SNAPSHOTS;
    ImGui::RadioButton("Continuous", (int*)&selectedMode, (int)vk::RecordMode::CONTINUOUS); ImGui::SameLine();
    ImGui::RadioButton("Snapshot", (int*)&selectedMode, (int)vk::RecordMode::SNAPSHOTS);

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