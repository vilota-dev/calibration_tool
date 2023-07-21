#include "recorder/presets.hpp"

namespace vk {
    std::vector<Preset>& getPresets() {
        // Make Presets for vk180, VKL, VK360 front and back
        static std::vector<Preset> presets;

        if (presets.empty()) {
            presets.emplace_back("VK180", "S0/", std::vector<std::string>{"camb", "camc", "camd"});
            presets.emplace_back("VKL", "S0/", std::vector<std::string>{"cama", "camb", "camc", "camd"});
            presets.emplace_back("VK360 front", "S0/", std::vector<std::string>{"camb", "camc", "camd"});
            presets.emplace_back("VK360 back", "S1/", std::vector<std::string>{"camb", "camc", "camd"});
        }

        return presets;
    }
}