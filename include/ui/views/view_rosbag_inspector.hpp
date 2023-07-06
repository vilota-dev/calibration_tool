#pragma once

#include "app_state.hpp"
#include "utils/utils.hpp"
//#include "utils/colors.h"
#include "ui/view.hpp"

#include "imgui.h"
#include "nfd.h"
#include "spdlog/spdlog.h"

#include <regex>
#include <string>
#include <map>
#include <string>

class ViewRosbagInspector : public View {
public:
    ViewRosbagInspector();
    ~ViewRosbagInspector() override = default;

    void draw_content() override;
    void draw_files();
    void draw_bag_content();
private:
    int selected_rosbag;
    std::map<std::string, uint64_t> num_topics_to_show;
};