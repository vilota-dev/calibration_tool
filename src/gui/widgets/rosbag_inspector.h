#pragma once

#include "utils/utils.h"
#include "gui/app_state.h"
#include "io/files_container.h"
#include <string>
#include <regex>

#include "imgui.h"
#include "nfd.h"
#include "spdlog/spdlog.h"

static const ImVec4 light_blue = from_rgba(0, 174, 239, 255, true); // Light blue color for selected elements such as play button glyph when paused
static const ImVec4 regular_blue = from_rgba(0, 115, 200, 255, true); // Checkbox mark, slider grabber
static const ImVec4 light_grey = from_rgba(0xc3, 0xd5, 0xe5, 0xff, true); // Text
static const ImVec4 dark_window_background = from_rgba(9, 11, 13, 255);
static const ImVec4 almost_white_bg = from_rgba(230, 230, 230, 255, true);
static const ImVec4 black = from_rgba(0, 0, 0, 255, true);
static const ImVec4 transparent = from_rgba(0, 0, 0, 0, true);
static const ImVec4 white = from_rgba(0xff, 0xff, 0xff, 0xff, true);
static const ImVec4 scrollbar_bg = from_rgba(14, 17, 20, 255);
static const ImVec4 scrollbar_grab = from_rgba(54, 66, 67, 255);
static const ImVec4 grey{ 0.5f,0.5f,0.5f,1.f };
static const ImVec4 dark_grey = from_rgba(30, 30, 30, 255);
static const ImVec4 sensor_header_light_blue = from_rgba(80, 99, 115, 0xff);
static const ImVec4 sensor_bg = from_rgba(36, 44, 51, 0xff);
static const ImVec4 redish = from_rgba(255, 46, 54, 255, true);
static const ImVec4 dark_red = from_rgba(200, 46, 54, 255, true);
static const ImVec4 button_color = from_rgba(62, 77, 89, 0xff);
static const ImVec4 header_window_bg = from_rgba(36, 44, 54, 0xff);
static const ImVec4 header_color = from_rgba(62, 77, 89, 255);
static const ImVec4 title_color = from_rgba(27, 33, 38, 255);
static const ImVec4 device_info_color = from_rgba(33, 40, 46, 255);
static const ImVec4 yellow = from_rgba(229, 195, 101, 255, true);
static const ImVec4 green = from_rgba(0x20, 0xe0, 0x20, 0xff, true);

int draw_files_left_panel(int flags, RosbagContainer &files, std::map<std::string, uint64_t> num_topics_to_show) {
  ImGui::BeginChild("Loaded Files", ImVec2(250, 0), true, flags);
  static int selected = 0;

  for (int i = 0; i < files.size(); i++)
  {
    ImGui::PushStyleColor(ImGuiCol_Text, white);
    ImGui::PushStyleColor(ImGuiCol_TextSelectedBg, white);
    ImGui::PushStyleColor(ImGuiCol_Header, grey);
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, light_grey);
    if (ImGui::Selectable(files[i]->get_file_path().c_str(), selected == i, 0, ImVec2(100, 0)))
    {
      selected = i;
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
      if (selected >= i)
        selected = std::max(0, selected - 1);
      i = next - 1; //since we will "i++" next
    }
  }
  ImGui::EndChild();

  return selected;
}

void draw_bag_content(std::shared_ptr<RosbagDataset> &dataset, int flags) {
  ImGui::BeginChild("Bag Content", ImVec2(0, 0), false, flags);
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

void draw_rosbag_inspector(AppState &app_state) {
  if (ImGui::Begin("ROS .bag Inspector", nullptr)) {
    static nfdfilteritem_t filterItem[1] = {{"ROS .bag file", "bag"}};

    app_state.selected = draw_files_left_panel(0, app_state.rosbag_files, app_state.num_topics_to_show);

    ImGui::SameLine();

    if (app_state.rosbag_files.size() > 0) {
      draw_bag_content(app_state.rosbag_files[app_state.selected], 0);
    }


  }
  ImGui::End();
}