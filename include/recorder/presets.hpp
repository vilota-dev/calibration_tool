#pragma once

#include <string>
#include <utility>

class Preset {
public:
    std::string name;
    std::string tf_prefix;
    std::vector<std::string> topics;

    Preset() = delete;

    Preset(std::string name, std::string tf_prefix, std::vector<std::string> topics)
        : name(std::move(name)), tf_prefix(std::move(tf_prefix)), topics(std::move(topics)) {}

    Preset(const Preset&) = default;
    Preset& operator=(const Preset&) = default;

    Preset(Preset&&) = default;
    Preset& operator=(Preset&&) = default;

    ~Preset() = default;
};

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