#pragma once

#include "calibration/aprilgrid.h"
#include "io/dataset_io.h"
#include "utils/common_types.h"

#include <basalt/utils/apriltag.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <spdlog/spdlog.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

namespace basalt {
  struct CalibCornerData {
    Eigen::aligned_vector<Eigen::Vector2d> corners;
    std::vector<int> corner_ids;
    std::vector<double> radii;  //!< threshold used for maximum displacement
    //! during sub-pix refinement; Search region is
    size_t seq; // sequence in the dataset
    //! slightly larger.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct ProjectedCornerData {
    Eigen::aligned_vector<Eigen::Vector2d> corners_proj;
    std::vector<bool> corners_proj_success;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct CalibInitPoseData {
    Sophus::SE3d T_a_c;
    size_t num_inliers;

    Eigen::aligned_vector<Eigen::Vector2d> reprojected_corners;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using CalibCornerMap = tbb::concurrent_unordered_map<TimeCamId, CalibCornerData,
          std::hash<TimeCamId>>;

  using CalibInitPoseMap =
          tbb::concurrent_unordered_map<TimeCamId, CalibInitPoseData,
                  std::hash<TimeCamId>>;

  class CalibHelper {
  public:
    static void detectCorners(const basalt::VioDatasetPtr &vio_data, const basalt::AprilGrid &april_grid,
                              basalt::CalibCornerMap &calib_corners,
                              basalt::CalibCornerMap &calib_corners_rejected) {
      spdlog::info("Started detecting corners");

      calib_corners.clear();
      calib_corners_rejected.clear();

      spdlog::info("Detecting corners with low ID = {}", april_grid.getLowId());

      tbb::parallel_for(
              tbb::blocked_range<size_t>(0, vio_data->get_image_timestamps().size()),
              [&](const tbb::blocked_range<size_t> &r) {
                const int numTags = april_grid.getTagCols() * april_grid.getTagRows();
                ApriltagDetector ad(numTags);

                for (size_t j = r.begin(); j != r.end(); ++j) {
                  int64_t timestamp_ns = vio_data->get_image_timestamps()[j];
                  const std::vector<ImageData> &img_vec =
                          vio_data->get_image_data(timestamp_ns);

                  for (size_t i = 0; i < img_vec.size(); i++) {
                    if (img_vec[i].img.get()) {
                      CalibCornerData ccd_good;
                      CalibCornerData ccd_bad;
                      ad.detectTags(*img_vec[i].img, ccd_good.corners,
                                    ccd_good.corner_ids, ccd_good.radii,
                                    ccd_bad.corners, ccd_bad.corner_ids, ccd_bad.radii);

                      spdlog::debug("image ({},{})  detected {} corners ({} rejected)",
                                    timestamp_ns, i, ccd_good.corners.size(),
                                    ccd_bad.corners.size());

                      TimeCamId tcid(timestamp_ns, i);

                      calib_corners.emplace(tcid, ccd_good);
                      calib_corners_rejected.emplace(tcid, ccd_bad);
                    }
                  }
                }
              });

      std::string path = "/Users/tejas/Developer/vilota-dev/calibration_tool/data/stuff_detected_corners.cereal";
      std::ofstream os(path, std::ios::binary);
      cereal::BinaryOutputArchive archive(os);

      archive(calib_corners);
      archive(calib_corners_rejected);

      std::cout << "Done detecting corners. Saved them here: " << path
                << std::endl;
    }

    static void initCamPoses(
            const Calibration<double>::Ptr &calib,
            const Eigen::aligned_vector<Eigen::Vector4d> &aprilgrid_corner_pos_3d,
            CalibCornerMap &calib_corners, CalibInitPoseMap &calib_init_poses);

    static bool initializeIntrinsics(
            const Eigen::aligned_vector<Eigen::Vector2d> &corners,
            const std::vector<int> &corner_ids, const AprilGrid &aprilgrid, int cols,
            int rows, Eigen::Vector4d &init_intr);

    static bool initializeIntrinsicsPinhole(
            const std::vector<CalibCornerData *> pinhole_corners,
            const AprilGrid &aprilgrid, int cols, int rows,
            Eigen::Vector4d &init_intr);

  private:
    inline static double square(double x) { return x * x; }

    inline static double hypot(double a, double b) {
      return sqrt(square(a) + square(b));
    }

    static void computeInitialPose(
            const Calibration<double>::Ptr &calib, size_t cam_id,
            const Eigen::aligned_vector<Eigen::Vector4d> &aprilgrid_corner_pos_3d,
            const basalt::CalibCornerData &cd, basalt::CalibInitPoseData &cp);

    static size_t computeReprojectionError(
            const UnifiedCamera<double> &cam_calib,
            const Eigen::aligned_vector<Eigen::Vector2d> &corners,
            const std::vector<int> &corner_ids,
            const Eigen::aligned_vector<Eigen::Vector4d> &aprilgrid_corner_pos_3d,
            const Sophus::SE3d &T_target_camera, double &error);
  };

}  // namespace basalt

namespace cereal {
  template<class Archive>
  void serialize(Archive &ar, basalt::CalibCornerData &c) {
    ar(c.corners, c.corner_ids, c.radii, c.seq);
  }

  template<class Archive>
  void serialize(Archive &ar, basalt::CalibInitPoseData &c) {
    ar(c.T_a_c, c.num_inliers, c.reprojected_corners);
  }
}  // namespace cereal