#include "ui/views/view_recorder.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>

ViewRecorder::ViewRecorder()
    : View("ROS .bag Dataset Recorder"),
    custom_camera(0),
    recorder_params(),
    selected_mode(vk::RecordMode::SNAPSHOT),
    dataset_recorder(),
    display_imgs(),
    display_params() {
    this->display_params.RefreshImage = true;
}

void ViewRecorder::draw_controls() {
    using namespace vk;
    ImGui::BeginChild("Recorder Controls", ImVec2(0, 200), true);

    ImGui::RadioButton("Preset", &this->custom_camera, 0); ImGui::SameLine();
    ImGui::RadioButton("Custom", &this->custom_camera, 1); ImGui::SameLine();

    // From better-enums documentation: You cannot convert a literal constant such as Channel::Cyan
    // directly to, for example, a string. You have to prefix it with a "+"
    int temp_selected = this->selected_mode;
    ImGui::RadioButton((+RecordMode::SNAPSHOT)._to_string(), &temp_selected, 0); ImGui::SameLine();
    ImGui::RadioButton((+RecordMode::CONTINUOUS)._to_string(), &temp_selected, 1);
    this->selected_mode = RecordMode::_from_integral(temp_selected);

    if (!this->custom_camera) {
        std::vector<Preset>& presets = getPresets();

        static int preset_current_idx = 0;
        const char* selectedPresetName = presets[preset_current_idx].name.c_str();
        if (ImGui::BeginCombo("Choose Custom Preset", selectedPresetName)) {
            for (int n = 0; n < presets.size(); n++) {
                const bool is_selected = (preset_current_idx == n);
                if (ImGui::Selectable(presets[n].name.c_str(), is_selected)) {
                    preset_current_idx = n;
                    recorder_params.camera_topics = presets[n].topics;
                    recorder_params.tf_prefix = presets[n].tf_prefix;
                }

                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    } else {
        // Add tf_prefix string input as well as cam_topics
        ImGui::InputText("tf_prefix", &recorder_params.tf_prefix);

        static char inputBuffer[256] = "";
        bool clicked = ImGui::InputText("New Topic", inputBuffer, IM_ARRAYSIZE(inputBuffer), ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        if (ImGui::Button("Add") || clicked) {
            recorder_params.camera_topics.emplace_back(inputBuffer);
            memset(inputBuffer, 0, sizeof(inputBuffer));
        }

        for (int i = 0; i < recorder_params.camera_topics.size(); ++i) {
            ImGui::Text("%s", recorder_params.camera_topics[i].c_str());

            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                // Remove the item from the list
                recorder_params.camera_topics.erase(recorder_params.camera_topics.begin() + i);
                --i;
            }
        }
    }
    /*
     * Draw the buttons for starting and stopping the recording
     * */
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.2f, 0.4f, 0.2f, 1.0f});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.2f, 0.6f, 0.2f, 1.0f});
        ImVec2 avail = ImGui::GetWindowSize();
        ImVec2 button_size = ImGui::CalcItemSize(ImVec2{avail.x / 2, 50}, 0.0f, 0.0f);

        if (ImGui::Button("Start Recording", button_size)) {
            this->dataset_recorder.init(this->recorder_params, this->selected_mode, this->display_imgs);
            dataset_recorder.start_record();
        }
        ImGui::PopStyleColor(2);

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.4f, 0.2f, 0.2f, 1.0f});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.6f, 0.2f, 0.2f, 1.0f});
        if (ImGui::Button("Stop Recording", button_size)) {
            dataset_recorder.stop_record();
        }
        ImGui::PopStyleColor(2);


        if (dataset_recorder.is_running()) {
            switch(this->dataset_recorder.get_mode()) {
                case RecordMode::CONTINUOUS:
                    ImGui::Text("Recording duration: "); // TODO: Add a timer
                    break;
                case RecordMode::SNAPSHOT:
                    if (ImGui::Button("Take snapshot") || ImGui::IsKeyPressed(ImGuiKey_Space)) {
                        dataset_recorder.take_snapshot();
                    } ImGui::SameLine();
                    ImGui::Text("Messages recorded: %i", dataset_recorder.get_num_snapshots());
                    break;
            }
        }
    }


    ImGui::EndChild();
}

void ViewRecorder::draw_cam_view() {
    ImGui::BeginChild("Live Camera View", ImVec2(0, 0), true);
    /*
     * Live view of the cameras
     * */
    {
        if (!this->dataset_recorder.is_running()) {
            // Note that 0 tells ImGui to "just use the default"
            ImVec2 button_size = ImGui::CalcItemSize(ImVec2{300, 0}, 0.0f, 0.0f);

            ImVec2 avail = ImGui::GetWindowSize();

            // Calculate center of window for button.
            ImVec2 centre_position_for_button{
                    // we have two widgets, so twice the size - and we need to account for the spacing in the middle
                    (avail.x - button_size.x) / 2,
                    (avail.y - button_size.y) / 2
            };

            // tell ImGui to render the button at the new position
            ImGui::SetCursorPos(centre_position_for_button);

            ImGui::Text("Press Start Recording to begin recording the dataset");
        } else {
            auto num_cams = this->recorder_params.camera_topics.size();

            ImGui::Columns(num_cams);

            int idx = 0;
            for (auto &cam_name: this->recorder_params.camera_topics) {
                this->display_params.ZoomKey = cam_name; // fix doesn't change
                ImmVision::Image(cam_name, display_imgs->at(idx), &this->display_params);
                ImGui::NextColumn();
                idx++;
            }
        }
    }
    ImGui::EndChild();
}

void ViewRecorder::draw_content() {
    if (!ImGui::Begin(this->get_name().c_str())) {
        ImGui::End();
        return;
    }
    using namespace vk;

    this->draw_controls();
    this->draw_cam_view();

    ImGui::End();
}