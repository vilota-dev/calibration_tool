#pragma once

#include "utils/utils.h"
#include "gui/app_state.h"
#include <string>
#include <regex>

#include "imgui.h"
#include "nfd.h"
#include "spdlog/spdlog.h"

int draw_files_left_panel(int flags) {
  ImGui::BeginChild("Loaded Files", ImVec2(150, 0), true, flags);
  static int selected = 0;
  // Needs the file container for this, just ignore first
  ImGui::EndChild();

  return selected;
}

void draw_bag_content(AppState app_state, int flags) {
  ImGui::BeginChild("Bag Content", ImVec2(0, 0), false, flags);
  static const ImVec4 white = from_rgba(0xff, 0xff, 0xff, 0xff, true);
  ImGui::PushStyleColor(ImGuiCol_Text, white);
  std::ostringstream oss;
  ImGui::Text("\t%s",
              std::string(tmpstringstream() << std::left << std::setw(20) << "Path: " << app_state.file_path).c_str());
  ImGui::Text("\t%s", std::string(
          tmpstringstream() << std::left << std::setw(20) << "Size: " << app_state.file_size << " MB").c_str());

  if (ImGui::CollapsingHeader("Topics")) {
    for (auto &&topic_to_message_type: app_state.topics_to_message_types) {
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
        rosbag::View messages(*app_state.rosbag_io.get()->get_bag(), rosbag::TopicQuery(topic));
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

void draw_rosbag_inspector(AppState &app_state) {
  if (ImGui::Begin("ROS .bag Inspector", nullptr)) {
    static nfdfilteritem_t filterItem[1] = {{"ROS .bag file", "bag"}};

    int selected = draw_files_left_panel(0);

    ImGui::SameLine();

    draw_bag_content(app_state, selected);
  }
  ImGui::End();
}