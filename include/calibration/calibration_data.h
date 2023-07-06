#pragma once

#include "calibration/aprilgrid.h"
#include "libcbdetect/config.h"
#include "utils/common_types.h"
#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/apriltag.h>
#include <tbb/concurrent_unordered_map.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

namespace basalt {
    enum class CalibType {
        Checkerboard_CBDETECT,
        Checkerboard_OpenCV,
        AprilGrid
    };

    struct OpenCVParams {
        int cols;
        int rows;
        double squareSpacingCols;
        double squareSpacingRows;
        bool adaptiveThresh;
        bool normalizeImage;
        bool filterQuads;
        bool fastCheck;
        // OpenCVParams(int width, int height, bool adaptiveThresh, bool normalizeImage, bool filterQuads, bool fastCheck) {
        //     this->width = width;
        //     this->height = height;
        //     this->adaptiveThresh = adaptiveThresh;
        //     this->normalizeImage = normalizeImage;
        //     this->filterQuads = filterQuads;
        //     this->fastCheck = fastCheck;
        // }

        // internal split/load function
        // https://uscilab.github.io/cereal/serialization_functions.html
        template<class Archive>
        void save(Archive &ar) const{
            ar(cereal::make_nvp("targetType", std::string("checkerboard"))); // convention https://github.com/ethz-asl/kalibr/wiki/calibration-targets
            ar(cereal::make_nvp("targetCols", cols));
            ar(cereal::make_nvp("targetRows", rows));
            ar(cereal::make_nvp("rowSpacingMeters", squareSpacingRows));
            ar(cereal::make_nvp("colSpacingMeters", squareSpacingCols));
        }

        template<class Archive>
        void load(Archive &ar){
            std::string targetType; 
            ar(cereal::make_nvp("targetType", targetType));
            if (targetType != "checkerboard")
                throw std::runtime_error("trying to serialise a checkerboard object with non checkerboard targetType");

            ar(cereal::make_nvp("targetType", std::string("checkerboard")));
            ar(cereal::make_nvp("targetCols", cols));
            ar(cereal::make_nvp("targetRows", rows));
            ar(cereal::make_nvp("rowSpacingMeters", squareSpacingRows));
            ar(cereal::make_nvp("colSpacingMeters", squareSpacingCols));
        }

        bool operator==(const OpenCVParams& rhs) const{
            std::stringstream os_lhs, os_rhs;

            // scope the serialisation
            {
                cereal::JSONOutputArchive ar_lhs(os_lhs);
                cereal::JSONOutputArchive ar_rhs(os_rhs);

                // ar_lhs(*this);
                // ar_rhs(rhs);

                this->save(ar_lhs);
                rhs.save(ar_rhs);
            }

            bool matched = (os_lhs.str() == os_rhs.str());

            if (matched) {
                spdlog::info("checkerboard param matched: \n{}", os_lhs.str());
                return true;
            }else {
                spdlog::info("checkerboard param mismatched \n{}\n---and---\n{}", os_lhs.str(), os_rhs.str());
                return false;
            }
        }

        bool operator!=(const OpenCVParams& rhs) const{
            return !(*this==rhs);
        }
    };

    struct CalibCornerData {
        Eigen::aligned_vector<Eigen::Vector2d> corners;
        std::vector<int> corner_ids;
        std::vector<double> radii;//!< threshold used for maximum displacement
        //! during sub-pix refinement; Search region is
        size_t seq;// no need to set in constructor, tbb loop does it
        //! slightly larger.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CalibCornerData() = default;

        CalibCornerData(const cbdetect::Corner &checkerboard_corner, const std::vector<cbdetect::Board> &boards) {
            for (const auto& board : boards) {
                int idx = 0;
                for(const auto & i : board.idx) {
                    for(int j = 0; j < i.size(); ++j) {
                        if(i[j] >= 0) { // if the corner is valid
                            int original_index = i[j];
                            this->corners.emplace_back(checkerboard_corner.p[original_index].x, checkerboard_corner.p[original_index].y);
                            this->corner_ids.push_back(idx++); // keep the original index
                            this->radii.push_back(checkerboard_corner.r[original_index]);
                        }
                    }
                }
            }
            // Lose the other fields
            // std::vector<cv::Point2d> v1;
            //  std::vector<cv::Point2d> v2;
            //  std::vector<cv::Point2d> v3;
            //  std::vector<double> score;
        }

        CalibCornerData(const std::vector<cv::Point2f> cv_corners) {
            int idx = 0;
            for(const auto & i : cv_corners) {
                this->corners.emplace_back(i.x, i.y);
                this->corner_ids.push_back(idx++);
                this->radii.push_back(6.0); // NEED CHANGE THIS!!
            }
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

    using CalibInitPoseMap =
            tbb::concurrent_unordered_map<TimeCamId, CalibInitPoseData,
                                          std::hash<TimeCamId>>;

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

        std::string getTargetType() {
            assert(targetType.empty());

            return targetType;
        }

    protected:
        std::string targetType;
    };

    class AprilGridParams : public CalibParams {
    public:
        AprilGridParams(const std::shared_ptr<AprilGrid> &april_grid) : ad(april_grid->getTagCols() * april_grid->getTagRows(), april_grid->getTagFamily(), april_grid->getLowId()) {
            this->april_grid = april_grid;
            targetType = "aprilgrid";
        }

        AprilGridParams() = delete;

        std::shared_ptr<AprilGrid> getParams() {return april_grid;}

        void process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

    private:
        std::shared_ptr<AprilGrid> april_grid; // serialisable object
        ApriltagDetector ad;
    };

    class CBCheckerboardParams : public CalibParams {
    public:
        CBCheckerboardParams(const std::shared_ptr<cbdetect::Params> &cb_params) {
            this->cb_params = cb_params;
            targetType = "checkerboard_cb";
        }

        CBCheckerboardParams() = delete;

        std::shared_ptr<cbdetect::Params> getParams() {return cb_params;}

        void process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

    protected:
        std::shared_ptr<cbdetect::Params> cb_params;
    };

    class OpenCVCheckerboardParams : public CalibParams {
    public:
        OpenCVCheckerboardParams(OpenCVParams& params) {

            cv_params = params;
            // Access prams to get flags
            this->flags += cv_params.adaptiveThresh ? cv::CALIB_CB_ADAPTIVE_THRESH : 0;
            this->flags += cv_params.filterQuads ? cv::CALIB_CB_FILTER_QUADS : 0;
            this->flags += cv_params.normalizeImage ? cv::CALIB_CB_NORMALIZE_IMAGE : 0;
            this->flags += cv_params.fastCheck ? cv::CALIB_CB_FAST_CHECK : 0;

            targetType = "checkerboard_opencv";
        }

        OpenCVCheckerboardParams() = delete;

        OpenCVParams getParams() {return cv_params;}

        void process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

    protected:
        OpenCVParams cv_params; // serialisable object
        int flags;
    };
}// namespace basalt

namespace cereal {
    template<class Archive>
    void serialize(Archive &ar, basalt::CalibCornerData &c) {
        ar(c.corners, c.corner_ids, c.radii, c.seq);
    }

    template<class Archive>
    void serialize(Archive &ar, basalt::CalibInitPoseData &c) {
        ar(c.T_a_c, c.num_inliers, c.reprojected_corners);
    }
}// namespace cereal