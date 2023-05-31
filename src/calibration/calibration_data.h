#include <basalt/utils/apriltag.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <tbb/concurrent_unordered_map.h>
#include "utils/common_types.h"

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
}