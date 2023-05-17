#include "calibration/aprilgrid.h"
#include "basalt/utils/apriltag.h"
#include "calibration/calibration_helper.h"
#include "io/dataset_io.h"
#include "spdlog/spdlog.h"
#include "basalt/serialization/headers_serialization.h"

#include <tbb/parallel_for.h>

using namespace basalt;

void detectCorners(const VioDatasetPtr &vio_data,
                   const AprilGrid &april_grid,
                   CalibCornerMap &calib_corners,
                   CalibCornerMap &calib_corners_rejected) {
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

//                  spdlog::info("image ({},{})  detected {} corners ({} rejected)",
//                               timestamp_ns, i, ccd_good.corners.size(),
//                               ccd_bad.corners.size());

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