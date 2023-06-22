#pragma once

#include "io/dataset_io.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/image_normalization_and_gradients.h"
#include "libcbdetect/get_init_location.h"
#include "libcbdetect/plot_corners.h"
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/filter_corners.h"
#include "libcbdetect/refine_corners.h"
#include "libcbdetect/polynomial_fit.h"

#include <spdlog/spdlog.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

namespace fs = std::filesystem;

namespace basalt {
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

      {
        std::ofstream os(this->cache_path, std::ios::binary);
        cereal::BinaryOutputArchive archive(os);

        archive(this->dataset->calib_corners);
        archive(this->dataset->calib_corners_rejected);
      }

      {
        std::ofstream os(this->cache_path_json, std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        archive(this->dataset->calib_corners);
        archive(this->dataset->calib_corners_rejected);
      }

      spdlog::info("Cached detected corners here: {}", this->cache_path.string());
    }

    void detectCorners(const std::shared_ptr<CalibParams> &params);


  protected:
    std::shared_ptr<RosbagDataset> dataset;
    std::shared_ptr<CalibParams> params;
    fs::path cache_path, cache_path_json;
  };
}