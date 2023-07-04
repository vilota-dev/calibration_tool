#pragma once

#include <string>
#include <utility>
#include <vector>

namespace vk {
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

    std::vector<Preset>& getPresets();
}