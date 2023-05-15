#include "io/dataset_io.h"
#include "io/dataset_io_rosbag.h"
#include "spdlog/spdlog.h"

namespace basalt {
    // Removed the other file formats for now, since we only need ROS .bag file formats
    DatasetIoInterfacePtr DatasetIoFactory::getDatasetIo(
            const std::string &dataset_type, bool load_mocap_as_gt) {
        if (dataset_type == "bag") {
            return DatasetIoInterfacePtr(new RosbagIO);
        } else {
            spdlog::error("Dataset type {} is not supported", dataset_type);
            std::abort();
        }
    }
}  // namespace basalt