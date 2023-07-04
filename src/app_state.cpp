#include "app_state.hpp"

#include "nfd.hpp"

void AppState::load_dataset() {
    NFD::Guard nfdGuard;
    NFD::UniquePath outPath;
    nfdfilteritem_t bagFilter[1] = {{"ROS .bag file", "bag"}}; // support for png later
    nfdresult_t result = NFD::OpenDialog(outPath, bagFilter, 1);

    if (result == NFD_OKAY) {
        const std::string path = outPath.get();
        AppState::get_instance().submit_task([this, path]() {
            if (path.find(".bag") != std::string::npos) {
                rosbag_files.addFiles(std::vector<std::string>{path});
                spdlog::debug("Success! File loaded from {}", path);
            } else {
                spdlog::error("Unknown file type: {}. Try re-uploading.", path);
            }
        });
    } else if (result == NFD_CANCEL) {
        spdlog::debug("User pressed cancel.");
    } else {
        spdlog::error("File upload failed. Error: {}", NFD_GetError());
    }
}