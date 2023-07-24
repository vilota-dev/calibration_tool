#pragma once

#include <ecal_camera/CameraInterface.hpp>

#include <spdlog/spdlog.h>

#include <string>
#include <utility>
#include <vector>
#include <set>

namespace vk {
    class Preset {
    public:
        Preset() = delete;

        // Takes in a vector of strings, with each string prefixed by tf_prefix.
        // We need to remove the tf_prefix and add the camera name to the end.
        // Then for each unique prefix, we need to create a new CameraParams object.
        // Then set the tf_prefix and camera_topics of the CameraParams object accordingly
        Preset(std::string name, std::vector<std::string> topics)
                : name(std::move(name)) {
            std::set<std::string> prefixes; // temp variable to store all unique prefixes
            for (const auto& topic : topics) {
                std::string prefix = topic.substr(0, topic.find_last_of('/') + 1);
                prefixes.insert(prefix);
            }

            for (const auto& prefix : prefixes) {
                vk::CameraParams param;
                param.tf_prefix = prefix;
                spdlog::trace("Created CameraParams with tf_prefix {}", param.tf_prefix);
                for (const auto& topic : topics) {
                    if (topic.substr(0, topic.find_last_of('/')) == prefix) {
                        size_t pos = topic.find('/');
                        param.camera_topics.push_back(topic.substr(pos+1));
                        spdlog::trace("Added topic {} to preset {}", topic.substr(pos+1),this->name);
                    }
                }
                this->num_cams += param.camera_topics.size();
                params.push_back(std::move(param));
            }

            spdlog::debug("Created preset {} with {} sets of cameras", this->name, prefixes.size());
        }

        Preset(const Preset&) = default;
        Preset& operator=(const Preset&) = default;

        Preset(Preset&&) = default;
        Preset& operator=(Preset&&) = default;

        ~Preset() = default;

        [[nodiscard]]
        const std::string& get_name() const { return this->name; }

        [[nodiscard]]
        const std::vector<vk::CameraParams>& get_params() const { return this->params; }

        [[nodiscard]]
        const int get_num_cams() const { return this->num_cams; }
    private:
        std::string name;
        std::vector<vk::CameraParams> params; // std::vector to contain all camera sets, e.g VK360 has 2 sets

        int num_cams = 0;
    };

    std::vector<Preset>& getPresets();
}