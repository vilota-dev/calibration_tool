#pragma once

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

        inline bool loadCache() {
            std::ifstream is(this->cache_path, std::ios::binary);

            spdlog::info("Attempting to load cache from {}", this->cache_path.string());

            if (is.good()) {
                cereal::BinaryInputArchive archive(is);

                try {
                    if (!load(archive, params))
                        return false;
                } catch (cereal::Exception &e) {
                    spdlog::error("Error from calibrator.hpp: {}", e.what());
                    return false;
                }

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
                save(archive, params);
            }

            {
                std::ofstream os(this->json_cache_path, std::ios::binary);
                cereal::JSONOutputArchive archive(os);
                save(archive, params);
            }

            spdlog::info("Cached detected corners here: {}", this->cache_path.string());
        }

        void detectCorners(const std::shared_ptr<CalibParams> &params);

        template<class Archive>
        bool load(Archive &ar, const std::shared_ptr<CalibParams> &params){

            const std::string calibBoardType = params->get_target_type();
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
                if (calibBoardType == "checkerboard_opencv") {
                    const auto& params_object = std::static_pointer_cast<CheckerboardParams>(params);

                    CheckerboardParams paramsCached({}); // temporary object to get cached config from json file
                    ar(cereal::make_nvp("calib_board_params", paramsCached));

                    if (paramsCached != paramsObject) {
                        return false;
                    }


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

            ar(cereal::make_nvp("corners", this->dataset->calib_corners));
            ar(cereal::make_nvp("corners_rejected", this->dataset->calib_corners_rejected));

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
                    const auto& paramsObject = std::static_pointer_cast<OpenCVCheckerboardParams>(params)->getParams();

                    ar(cereal::make_nvp("calib_board_params", paramsObject));

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
        std::shared_ptr<RosbagDataset> dataset;
        std::shared_ptr<CalibParams> params;
        fs::path cache_path;
        fs::path json_cache_path;
    };
}