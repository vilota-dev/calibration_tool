#pragma once

#include "utils/utils.h"
#include "dataset_io/dataset_io.h"

#include "imgui.h"
#include "nfd.h"
#include "spdlog/spdlog.h"

void draw_menu_bar(ROSbag &bag, nfdresult_t &result, nfdchar_t *outPath) {
    static nfdfilteritem_t filterItem[1] = {{"ROS .bag file", "bag"}};

    ImGui::BeginMenuBar();
    if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Load .bag file")) {
            result = NFD_OpenDialog(&outPath, filterItem, 1, NULL);
            if (result == NFD_OKAY) {
                spdlog::info("Success!");
                spdlog::info("File is located here: {}", outPath);
                // call the load_bag function on the bag object
                bag.load_bag(outPath);
                NFD_FreePath(outPath);
            } else if (result == NFD_CANCEL) {
                spdlog::info("User pressed cancel.");
            } else {
                spdlog::error("Error: {}", NFD_GetError());
            }
        }
        ImGui::EndMenu();
    }

    ImGui::EndMenuBar();
}

int draw_files_left_panel(int flags) {
    ImGui::BeginChild("Loaded Files", ImVec2(150, 0), true, flags);
    static int selected = 0;
    // Needs the file container for this, just ignore first
    ImGui::EndChild();

    return selected;
}

void draw_bag_content(ROSbag& bag, int flags) {
    ImGui::BeginChild("Bag Content", ImVec2(0, 0), false, flags);
    static const ImVec4 white = from_rgba(0xff, 0xff, 0xff, 0xff, true);
    ImGui::PushStyleColor(ImGuiCol_Text, white);
    std::ostringstream oss;
    ImGui::Text("\t%s", std::string(tmpstringstream() << std::left << std::setw(20) << "Path: " << bag.file_path).c_str());
    ImGui::Text("\t%s", std::string(tmpstringstream() << std::left << std::setw(20) << "Bag Version: " << bag.file_version).c_str());
    ImGui::Text("\t%s", std::string(tmpstringstream() << std::left << std::setw(20) << "Size: " << bag.file_size << " MB").c_str());

    if (ImGui::CollapsingHeader("Topics"))
    {
        for (auto&& topic_to_message_type : bag.topics_to_message_types)
        {
            std::string topic = topic_to_message_type.first;
            std::vector<std::string> messages_types = topic_to_message_type.second;
            std::ostringstream oss;
            int max_topic_len = 100;
            oss << std::left << std::setw(max_topic_len) << topic
                << " " << std::left << std::setw(10) << messages_types.size() << std::setw(6) << std::string(" msg") + (messages_types.size() > 1 ? "s" : "")
                << ": " << std::left << std::setw(40) << messages_types.front() << std::endl;
            std::string line = oss.str();
            auto pos = ImGui::GetCursorPos();
            ImGui::SetCursorPos({ pos.x + 20, pos.y });
            if (ImGui::CollapsingHeader(line.c_str()))
            {
                rosbag::View messages(bag.bag, rosbag::TopicQuery(topic));
                uint64_t count = 0;
                constexpr uint64_t num_next_items_to_show = 10;
//                num_topics_to_show[topic] = std::max(num_topics_to_show[topic], num_next_items_to_show);
                uint64_t max = 10;
                auto win_pos = ImGui::GetWindowPos();
                ImGui::SetWindowPos({ win_pos.x + 20, win_pos.y });
                for (auto&& m : messages)
                {
                    count++;
                    ImGui::Columns(2, "Message", true);
                    ImGui::Separator();
                    ImGui::Text("Timestamp"); ImGui::NextColumn();
                    ImGui::Text("Content"); ImGui::NextColumn();
                    ImGui::Separator();
//                    ImGui::Text("%s", pretty_time(std::chrono::nanoseconds(m.getTime().toNSec())).c_str()); ImGui::NextColumn();
//                    ImGui::Text("%s", bag.instanciate_and_cache(m, count).c_str());
                    ImGui::Columns(1);
                    ImGui::Separator();
                    if (count >= max)
                    {
                        int left = messages.size() - max;
                        if (left > 0)
                        {
                            ImGui::Text("... %d more messages", left);
                            ImGui::SameLine();
                            std::string label = tmpstringstream() << "Show More ##" << topic;
                        }
                    }
                }
                ImGui::SetWindowPos(win_pos);
            }

            if (ImGui::IsItemHovered())
            {
                if (topic.size() > max_topic_len)
                {
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