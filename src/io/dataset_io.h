#pragma once

#include <array>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/bitset.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

#include <Eigen/Dense>
#include <basalt/utils/sophus_utils.hpp>

#include <basalt/image/image.h>
#include <basalt/utils/assert.h>

#include <basalt/camera/generic_camera.hpp>
#include <basalt/camera/stereographic_param.hpp>

namespace basalt {

    struct ImageData {
        ImageData() : exposure(0) {}

        ManagedImage<uint16_t>::Ptr img;
        double exposure;
    };

    struct Observations {
        Eigen::aligned_vector<Eigen::Vector2d> pos;
        std::vector<int> id;
    };

    struct GyroData {
        int64_t timestamp_ns;
        Eigen::Vector3d data;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct AccelData {
        int64_t timestamp_ns;
        Eigen::Vector3d data;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PoseData {
        int64_t timestamp_ns;
        Sophus::SE3d data;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MocapPoseData {
        int64_t timestamp_ns;
        Sophus::SE3d data;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct AprilgridCornersData {
        int64_t timestamp_ns;
        int cam_id;

        Eigen::aligned_vector<Eigen::Vector2d> corner_pos;
        std::vector<int> corner_id;
    };

    class VioDataset {
    public:
        virtual ~VioDataset(){};

        virtual double get_size() const = 0;
        virtual size_t get_num_cams() const = 0;
        virtual std::set<std::string> get_camera_names() = 0;
        virtual std::string get_imu_name() = 0;

        virtual std::vector<int64_t> &get_image_timestamps() = 0;

        virtual const Eigen::aligned_vector<AccelData> &get_accel_data() const = 0;
        virtual const Eigen::aligned_vector<GyroData> &get_gyro_data() const = 0;
        virtual const std::vector<int64_t> &get_gt_timestamps() const = 0;
        virtual const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data()
        const = 0;
        virtual int64_t get_mocap_to_imu_offset_ns() const = 0;
        virtual std::vector<ImageData> get_image_data(int64_t t_ns) = 0;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::shared_ptr<VioDataset> VioDatasetPtr;

    class DatasetIoInterface {
    public:
        virtual void read(const std::string &path) = 0;
        virtual void reset() = 0;
        virtual VioDatasetPtr get_data() = 0;

        virtual ~DatasetIoInterface(){};
    };

    typedef std::shared_ptr<DatasetIoInterface> DatasetIoInterfacePtr;

    class DatasetIoFactory {
    public:
        static DatasetIoInterfacePtr getDatasetIo(const std::string &dataset_type,
                                                  bool load_mocap_as_gt = false);
    };

}  // namespace basalt

namespace cereal {

    template <class Archive>
    void serialize(Archive &archive, basalt::ManagedImage<uint8_t> &m) {
        archive(m.w);
        archive(m.h);

        m.Reinitialise(m.w, m.h);

        archive(binary_data(m.ptr, m.size()));
    }

    template <class Archive>
    void serialize(Archive &ar, basalt::GyroData &c) {
        ar(c.timestamp_ns, c.data);
    }

    template <class Archive>
    void serialize(Archive &ar, basalt::AccelData &c) {
        ar(c.timestamp_ns, c.data);
    }

}  // namespace cereal