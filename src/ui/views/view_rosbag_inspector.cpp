#include "ui/views/view_rosbag_inspector.hpp"

#include <imgui_internal.h>
#include <spdlog/spdlog.h>
#include <tracy/Tracy.hpp>

static const ImVec4 white = from_rgba(0xff, 0xff, 0xff, 0xff, true);
static const ImVec4 grey{ 0.5f,0.5f,0.5f,1.f };
static const ImVec4 light_grey = from_rgba(0xc3, 0xd5, 0xe5, 0xff, true); // Text

ViewRosbagInspector::ViewRosbagInspector() : View("Rosbag Inspector") {

}

void ViewRosbagInspector::draw_content() {
    if (!ImGui::Begin(this->get_name().c_str())) {
        ImGui::End();
        return;
    }
    ZoneScopedN("ViewRosbagInspector::draw_content");

    this->draw_files(); ImGui::SameLine();
    this->draw_bag_content();

    ImGui::End();
}

void ViewRosbagInspector::draw_files() {
    ImGui::BeginChild("Loaded Files", ImVec2(250, 0), true, 0);
    auto &app_state = AppState::get_instance();
    auto &files = app_state.rosbag_files;

    for (int i = 0; i < files.size(); i++)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, white);
        ImGui::PushStyleColor(ImGuiCol_TextSelectedBg, white);
        ImGui::PushStyleColor(ImGuiCol_Header, grey);
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, light_grey);

        if (ImGui::Selectable(files[i]->get_file_path().c_str(), this->selected_rosbag == i, 0, ImVec2(100, 0))) {
            this->selected_rosbag = i;
            num_topics_to_show.clear();
        }
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(450.0f);
            ImGui::TextUnformatted("Right click for more options");
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
        std::string label = tmpstringstream() << "file operations " << files[i]->get_file_path().c_str();
        if (ImGui::BeginPopupContextItem(label.c_str()))
        {
            ImGui::Text("Choose operation");
            ImGui::Separator();
            if (ImGui::BeginMenu("Sort by"))
            {
                if (ImGui::MenuItem("Hardware Timestamp")) {}
                if (ImGui::MenuItem("Arrival Timestamp")) {}
                if (ImGui::MenuItem("System Timestamp")) {}
                if (ImGui::MenuItem("Capture Timestamp")) {}
                ImGui::EndMenu();
            }
            ImGui::EndPopup();
        }
        label = "x##" + files[i]->get_file_path();
        ImGui::SameLine();
        ImGui::SetCursorPosX(120);
        if (ImGui::SmallButton(label.c_str()))
        {
            int next = files.remove_file(i);
            if (this->selected_rosbag >= i)
                this->selected_rosbag = std::max(0, this->selected_rosbag - 1);
            i = next - 1; //since we will "i++" next
        }
    }
    ImGui::EndChild();
}

void ViewRosbagInspector::draw_bag_content() {
    ImGui::BeginChild("Bag Content", ImVec2(0, 0), true, 0);
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
        return;
    }


    auto &dataset = app_state.rosbag_files[this->selected_rosbag];

    ImGui::PushStyleColor(ImGuiCol_Text, white);
    std::ostringstream oss;
    ImGui::Text("\t%s",
                std::string(tmpstringstream() << std::left << std::setw(20) << "Path: " << dataset->get_file_path()).c_str());
    ImGui::Text("\t%s", std::string(
            tmpstringstream() << std::left << std::setw(20) << "Size: " << dataset->get_file_size() << " mb").c_str());

    if (ImGui::CollapsingHeader("Topics")) {
        for (auto &&topic_to_message_type: dataset->topics_to_message_types) {
            std::string topic = topic_to_message_type.first;
            std::vector<std::string> messages_types = topic_to_message_type.second;
            std::ostringstream oss;
            int max_topic_len = 100;
            oss << std::left << std::setw(max_topic_len) << topic
                << " " << std::left << std::setw(10) << messages_types.size() << std::setw(6)
                << std::string(" msg") + (messages_types.size() > 1 ? "s" : "")
                << ": " << std::left << std::setw(40) << messages_types.front() << std::endl;
            std::string line = oss.str();
            auto pos = ImGui::GetCursorPos();
            ImGui::SetCursorPos({pos.x + 20, pos.y});

            if (ImGui::CollapsingHeader(line.c_str())) {
                rosbag::View messages(*dataset->get_bag(), rosbag::TopicQuery(topic));
                uint64_t count = 0;
                constexpr uint64_t num_next_items_to_show = 10;
                static uint64_t max = 10;
                auto win_pos = ImGui::GetWindowPos();
                ImGui::SetWindowPos({win_pos.x + 20, win_pos.y});

                for (auto &&m: messages) {
                    count++;
                    ImGui::Columns(2, "Message", true);
                    ImGui::Separator();
                    ImGui::Text("Timestamp");
                    ImGui::NextColumn();
                    ImGui::Text("Content");
                    ImGui::NextColumn();
                    ImGui::Separator();
                    ImGui::Text("%s", pretty_time(std::chrono::nanoseconds(m.getTime().toNSec())).c_str()); ImGui::NextColumn();
//          ImGui::Text("%s", get_message(m).c_str());
                    ImGui::Columns(1);
                    ImGui::Separator();
                    if (count >= max) {
                        int left = messages.size() - max;
                        if (left > 0) {
                            ImGui::Text("... %d more messages", left);
                            ImGui::SameLine();
                            std::string label = tmpstringstream() << "Show More ##" << topic;
                            if (ImGui::Button(label.c_str()))
                            {
                                max += 10;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                ImGui::SetWindowPos(win_pos);
            }

            if (ImGui::IsItemHovered()) {
                if (topic.size() > max_topic_len) {
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(450.0f);
                    ImGui::Text("%s", topic.c_str());
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                }
            }

        }
    }
    ImGui::PopStyleColor();
    ImGui::EndChild();
}