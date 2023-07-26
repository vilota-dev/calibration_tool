#include "recorder/presets.hpp"

namespace vk {
    std::vector<Preset>& getPresets() {
        // Make Presets for vk180, VKL, VK360 front and back
        static std::vector<Preset> presets;

        if (presets.empty()) {
            presets.emplace_back("VK180", std::vector<std::string>{"S0/camb", "S0/camc", "S0/camd"});
            presets.emplace_back("VKL", std::vector<std::string>{"S0/cama", "S0/camb", "S0/camc", "S0/camd"});
            presets.emplace_back("VK360 front", std::vector<std::string>{"S0/camb", "S0/camc", "S0/camd"});
            presets.emplace_back("VK360 back", std::vector<std::string>{"S1/camb", "S1/camc", "S1/camd"});
            presets.emplace_back("VK360 (full)", std::vector<std::string>{"S0/camb", "S0/camc", "S0/camd", "S1/camb", "S1/camc", "S1/camd"});
            presets.emplace_back("Fisheye 360", std::vector<std::string>{"S0/camd", "S1/camd"});
            presets.emplace_back("WFOV 360", std::vector<std::string>{"S0/camb", "S0/camc", "S1/camb", "S1/camc"});
        }

        return presets;
    }
}