#pragma once

#include "calibration/calibration_data.hpp"

#include "io/dataset_io.h"

#include <spdlog/spdlog.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

namespace fs = std::filesystem;

namespace basalt {
  class Calibrator {
  public:
    Calibrator(const std::shared_ptr<RosbagDataset> &dataset);

    ~Calibrator() = default;

    // loading from cache will not override params of target calibration board
    bool loadCache(const std::shared_ptr<CalibParams> &params) {

      std::ifstream is(this->m_cachePath, std::ios::binary);

      spdlog::info("loading cache from {}", this->m_cachePath.string());

      if (is.good()) {
        cereal::BinaryInputArchive archive(is);

        try {
          if (!load(archive, params))
            return false;
        }catch (cereal::Exception& e) {
          spdlog::error(e.what());
          return false;
        }

        spdlog::info("Loaded matched cached corners into memory, from: {}", this->m_cachePath.string());
        return true;
        
      }else {
        spdlog::error("fail to load cache file {}", this->m_cachePath.string());
        return false;
      }

    }

    inline void saveCache(const std::shared_ptr<CalibParams> &params) {

      {
        std::ofstream os(this->m_cachePath, std::ios::binary);
        cereal::BinaryOutputArchive archive(os);

        save(archive, params);
      }

      {
        std::ofstream os(this->m_cachePathJson, std::ios::binary);
        cereal::JSONOutputArchive archive(os);

        save(archive, params);
      }

      spdlog::info("Cached detected corners here: {}", this->m_cachePath.string());
    }

    void detectCorners(const std::shared_ptr<CalibParams> &params);

    template<class Archive>
    bool load(Archive &ar, const std::shared_ptr<CalibParams> &params){

      const std::string calibBoardType = params->getTargetType();
      std::string calibBoardTypeCached;

      // constexpr bool isLoading = std::is_base_of_v<cereal::detail::InputArchiveBase, Archive>;

      //// STEP 1 - board type
      {
        // NOTE: we have to put calib board type and params first
        // only when both matches, we shall process to loading
        ar(cereal::make_nvp("calib_board_type", calibBoardTypeCached));

        spdlog::info("loading cache: calib board type is {}, expected {}", calibBoardTypeCached, calibBoardType);

        if (calibBoardTypeCached != calibBoardType)
          return false;
      }

      //// STEP 2 - board params
      {
        if (calibBoardType == "checkerboard_cb") {
          throw std::runtime_error("not implemented save serialisation for cb type");
          // const auto paramsObjectPtr = std::static_pointer_cast<CBCheckerboardParams>(this->m_calibBoardParams)->getParams();
          // ar(cereal::make_nvp("calib_board_params", *paramsObjectPtr));
        }else if (calibBoardType == "checkerboard_opencv") {
          throw std::runtime_error("not implemented save serialisation for cv type");
          // const auto& paramsObject = std::static_pointer_cast<OpenCVCheckerboardParams>(this->m_calibBoardParams)->getParams();
          // ar(cereal::make_nvp("calib_board_params", paramsObject);
        }else if (calibBoardType == "aprilgrid") {
          const auto paramsObjectPtr = std::static_pointer_cast<AprilGridParams>(params)->getParams();
          AprilGrid paramsCached;
          ar(cereal::make_nvp("calib_board_params", paramsCached));

          if (paramsCached != *paramsObjectPtr) {
            return false;
          }

        }else {
          spdlog::error("loading cache: unsupported target type: " + calibBoardType);
          return false;
        }

      }

      //// CHECK passes, loading corners data

      ar(cereal::make_nvp("corners", this->m_dataset->calib_corners));
      ar(cereal::make_nvp("corners_rejected", this->m_dataset->calib_corners_rejected));

      return true;
    }

    template<class Archive>
    void save(Archive &ar, const std::shared_ptr<CalibParams> &params) const{

      //// STEP 1 - board type
      const std::string calibBoardType = params->getTargetType();
      ar(cereal::make_nvp("calib_board_type", calibBoardType));

      //// STEP 2 - board params
      {
        if (calibBoardType == "checkerboard_cb") {
          throw std::runtime_error("not implemented save serialisation for cb type");
          // const auto paramsObjectPtr = std::static_pointer_cast<CBCheckerboardParams>(this->m_calibBoardParams)->getParams();
          // ar(cereal::make_nvp("calib_board_params", *paramsObjectPtr));
        }else if (calibBoardType == "checkerboard_opencv") {
          throw std::runtime_error("not implemented save serialisation for cv type");
          // const auto& paramsObject = std::static_pointer_cast<OpenCVCheckerboardParams>(this->m_calibBoardParams)->getParams();
          // ar(cereal::make_nvp("calib_board_params", paramsObject);
        }else if (calibBoardType == "aprilgrid") {
          const auto paramsObjectPtr = std::static_pointer_cast<AprilGridParams>(params)->getParams();
          ar(cereal::make_nvp("calib_board_params", *paramsObjectPtr));

        }else {
          throw std::runtime_error("loading cache: unsupported target type: " + calibBoardType);
        }
      }

      ar(cereal::make_nvp("corners", this->m_dataset->calib_corners));
      ar(cereal::make_nvp("corners_rejected", this->m_dataset->calib_corners_rejected));

    }


  protected:
    std::shared_ptr<RosbagDataset> m_dataset;
    // std::shared_ptr<CalibParams> m_calibBoardParams;
    // std::string m_calibBoardType;
    fs::path m_cachePath, m_cachePathJson;
  };
}