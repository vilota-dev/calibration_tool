#pragma once

#include "calibration/aprilgrid.h"
#include "io/dataset_io.h"

#include <spdlog/spdlog.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

namespace basalt {
  class CalibHelper {
  public:
    static void detectCorners(const std::shared_ptr<RosbagDataset> &vio_data,
                              const std::shared_ptr<basalt::AprilGrid> &april_grid) {
      // Eventually, remove this to be not hard coded.
      std::string path = "/Users/tejas/Developer/vilota-dev/calibration_tool/data/stuff_detected_corners.cereal";

      std::ifstream is(path, std::ios::binary);

      if (is.good()) {
        cereal::BinaryInputArchive archive(is);

        vio_data->calib_corners.clear();
        vio_data->calib_corners_rejected.clear();
        archive(vio_data->calib_corners);
        archive(vio_data->calib_corners_rejected);

        spdlog::info("Loaded detected corners from: {}", path);
        return;
      }

      spdlog::info("No pre-processed detected corners found, started corner detection");

      vio_data->calib_corners.clear();
      vio_data->calib_corners_rejected.clear();

      spdlog::debug("Detecting corners with low ID = {}", april_grid->getLowId());

      tbb::parallel_for(
              tbb::blocked_range<size_t>(0, vio_data->get_image_timestamps().size()),
              [&](const tbb::blocked_range<size_t> &r) {
                const int numTags = april_grid->getTagCols() * april_grid->getTagRows();
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

//                      calib_corners.emplace(tcid, ccd_good);
                      vio_data->calib_corners_rejected.emplace(tcid, ccd_bad);
                    }
                  }
                }
              });

      std::ofstream os(path, std::ios::binary);
      cereal::BinaryOutputArchive archive(os);

      archive(vio_data->calib_corners);
      archive(vio_data->calib_corners_rejected);

      spdlog::info("Done detecting corners. Saved them here: {}", path);
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