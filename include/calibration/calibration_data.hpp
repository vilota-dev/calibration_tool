#pragma once

#include "calibration/aprilgrid.hpp"
#include "utils/common_types.h"
#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/apriltag.h>
#include <tbb/concurrent_unordered_map.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

namespace basalt {
    struct CalibCornerData {
        Eigen::aligned_vector<Eigen::Vector2d> corners;
        std::vector<int> corner_ids;
        std::vector<double> radii;//!< threshold used for maximum displacement
        //! during sub-pix refinement; Search region is
        size_t seq;// no need to set in constructor, tbb loop does it
        //! slightly larger.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CalibCornerData() = default;

        CalibCornerData(const std::vector<cv::Point2f> cv_corners) {
            int idx = 0;
            for (const auto &i: cv_corners) {
                this->corners.emplace_back(i.x, i.y);
                this->corner_ids.push_back(idx++);
                this->radii.push_back(1.0); // NEED CHANGE THIS!!
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
        CalibParams(std::string name) : target_type(name) {}
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

        inline const std::string &get_target_type() const { return target_type; }

    protected:
        std::string target_type;
    };

    class AprilGridParams : public CalibParams {
    public:
        explicit AprilGridParams(const std::shared_ptr<AprilGrid> &april_grid) : CalibParams("aprilgrid"), ad(
                april_grid->getTagCols() * april_grid->getTagRows(), april_grid->getTagFamily(),
                april_grid->getLowId()) {
            this->april_grid = april_grid;
        }

        AprilGridParams() = delete;

        std::shared_ptr<AprilGrid> getParams() { return april_grid; }

        void
        process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

    private:
        std::shared_ptr<AprilGrid> april_grid;
        ApriltagDetector ad;
    };

    class CheckerboardParams : public CalibParams {
    public:
        CheckerboardParams(int cols, int rows, bool adaptive_thresh, bool normalize_image,
                                 bool filter_quads, bool fast_check, bool enable_subpix_refine)
                                 : CalibParams("checkerboard"),
                                 cols(cols), rows(rows), flags(0), enable_subpix_refine(enable_subpix_refine) {
            // Access prams to get flags
            this->flags += adaptive_thresh ? cv::CALIB_CB_ADAPTIVE_THRESH : 0;
            this->flags += filter_quads ? cv::CALIB_CB_FILTER_QUADS : 0;
            this->flags += normalize_image ? cv::CALIB_CB_NORMALIZE_IMAGE : 0;
            this->flags += fast_check ? cv::CALIB_CB_FAST_CHECK : 0;
            this->enable_subpix_refine = enable_subpix_refine;
        }

        CheckerboardParams() = delete;

        void
        process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) override;

    protected:
        int cols;
        int rows;
        int square_spacing_cols;
        int square_spacing_rows;

        int flags;
        bool enable_subpix_refine;

    public:
        /*
         * Below is the logic for the serialisation
         * */

        /*
         * internal split/load function
         * https://uscilab.github.io/cereal/serialization_functions.html
         * */
        template<class Archive>
        void save(Archive &ar) const{
            ar(cereal::make_nvp("targetType", this->target_type)); // convention https://github.com/ethz-asl/kalibr/wiki/calibration-targets
            ar(cereal::make_nvp("targetCols", this->cols));
            ar(cereal::make_nvp("targetRows", this->rows));
            ar(cereal::make_nvp("rowSpacingMeters", this->square_spacing_rows));
            ar(cereal::make_nvp("colSpacingMeters", this->square_spacing_cols));
        }

        template<class Archive>
        void load(Archive &ar){
            std::string targetType;
            ar(cereal::make_nvp("targetType", targetType));
            if (targetType != "checkerboard")
                throw std::runtime_error("trying to serialise a checkerboard object with non checkerboard targetType");

            ar(cereal::make_nvp("targetType", this->target_type));
            ar(cereal::make_nvp("targetCols", this->cols));
            ar(cereal::make_nvp("targetRows", this->rows));
            ar(cereal::make_nvp("rowSpacingMeters", this->square_spacing_rows));
            ar(cereal::make_nvp("colSpacingMeters", this->square_spacing_cols));
        }

        bool operator==(const CheckerboardParams& rhs) const{
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

        bool operator!=(const CheckerboardParams& rhs) const{
            return !(*this==rhs);
        }
    };
}// namespace basalt

namespace cereal {
    template<class Archive>
    void serialize(Archive &ar, basalt::CalibCornerData &c) {
        ar(c.corners, c.corner_ids, c.radii, c.seq);
    }
}// namespace cereal