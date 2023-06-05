#include <basalt/utils/apriltag.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <tbb/concurrent_unordered_map.h>
#include "utils/common_types.h"
#include "libcbdetect/config.h"

// Store the structs relevant to calibration data in this file.
namespace basalt {
  struct CalibCornerData {
    Eigen::aligned_vector<Eigen::Vector2d> corners;
    std::vector<int> corner_ids;
    std::vector<double> radii;  //!< threshold used for maximum displacement
    //! during sub-pix refinement; Search region is
    size_t seq; // sequence in the dataset
    //! slightly larger.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CalibCornerData() = default;

    CalibCornerData(const cbdetect::Corner &checkerboard_corner, size_t seq) : seq(seq) {
      for (const auto &corner : checkerboard_corner.p) {
        corners.emplace_back(corner.x, corner.y);
      }
      // corner_ids = checkerboard_corner.ids; /* Leave it empty and get IDs through loop index*/
      for (const auto &radius : checkerboard_corner.r) {
        radii.push_back(radius);
      }
      // Lose the other fields
      // std::vector<cv::Point2d> v1;
      //  std::vector<cv::Point2d> v2;
      //  std::vector<cv::Point2d> v3;
      //  std::vector<double> score;
    }
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

  using CheckerboardCornerMap = tbb::concurrent_unordered_map<TimeCamId, cbdetect::Corner,
  std::hash<TimeCamId>>;

  using CalibInitPoseMap =
          tbb::concurrent_unordered_map<TimeCamId, CalibInitPoseData,
          std::hash<TimeCamId>>;
}

namespace cereal {
  template<class Archive, class T>
  void save(Archive& ar, const cv::Point_<T>& point) {
    ar << point.x << point.y;
  }

  template<class Archive, class T>
  void load(Archive& ar, cv::Point_<T>& point) {
    ar >> point.x >> point.y;
  }
}