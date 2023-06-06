#pragma once

#include "calibration/aprilgrid.h"
#include "io/dataset_io.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/image_normalization_and_gradients.h"
#include "libcbdetect/get_init_location.h"
#include "libcbdetect/plot_corners.h"
#include "libcbdetect/filter_corners.h"
#include "libcbdetect/refine_corners.h"
#include "libcbdetect/polynomial_fit.h"

#include <spdlog/spdlog.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

namespace fs = std::filesystem;

namespace basalt {
  class CalibParams {
  public:
    virtual ~CalibParams() = default;

    /*
     * process method will be called in the detectCorners loop for each ManagedImage.
     * AprilGrid detection right now takes in a Managed image, but preferably we can make the tag detector take in
     * cv::Mat instead, since we have preprocessed the cv::Mat.
     * Checkboard corner detection takes in cv::Mat, so currently it's converting to a cv::Mat before passing the
     * image to the find_corners function.
     * */
    virtual void
    process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) = 0;
  };

  class AprilGridParams : public CalibParams {
  public:
    AprilGridParams(const std::shared_ptr<AprilGrid> &april_grid) : ad(
            april_grid->getTagCols() * april_grid->getTagRows()) {
      this->april_grid = april_grid;
    }

    void process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

  private:
    std::shared_ptr<AprilGrid> april_grid;
    ApriltagDetector ad;
  };

  class CheckerboardParams : public CalibParams {
  public:
    CheckerboardParams(const std::shared_ptr<cbdetect::Params> &cb_params) {
      this->cb_params = cb_params;
    }

    void process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

  protected:
    std::shared_ptr<cbdetect::Params> cb_params;
  };

  class Calibrator {
  public:
    Calibrator(const std::shared_ptr<RosbagDataset> &dataset);

    ~Calibrator() = default;

    inline bool loadCache() {
      std::ifstream is(this->cache_path, std::ios::binary);

      if (is.good()) {
        cereal::BinaryInputArchive archive(is);

        this->dataset->calib_corners.clear();
        this->dataset->calib_corners_rejected.clear();
        archive(this->dataset->calib_corners);
        archive(this->dataset->calib_corners_rejected);

        spdlog::info("Loaded cached corners into memory, from: {}", this->cache_path.string());
        return true;
      }
      return false;
    }

    inline void saveCache() {
      std::ofstream os(this->cache_path, std::ios::binary);
      cereal::BinaryOutputArchive archive(os);

      archive(this->dataset->calib_corners);
      archive(this->dataset->calib_corners_rejected);

      spdlog::info("Cached detected corners here: {}", this->cache_path.string());
    }

    void detectCorners(const std::shared_ptr<CalibParams> &params);


  protected:
    std::shared_ptr<RosbagDataset> dataset;
    std::shared_ptr<CalibParams> params;
    fs::path cache_path;
  };
}