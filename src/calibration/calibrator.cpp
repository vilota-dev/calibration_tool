#include "calibration/calibrator.hpp"

namespace basalt {
    void AprilGridParams::process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) {
        ad.detectTags(img_raw, ccd_good.corners,
                      ccd_good.corner_ids, ccd_good.radii,
                      ccd_bad.corners, ccd_bad.corner_ids, ccd_bad.radii);
    }

    void CheckerboardParams::process(basalt::ManagedImage<uint16_t> &img_raw, CalibCornerData &ccd_good, CalibCornerData &ccd_bad) {
        // Convert img_raw into cv::Mat
        cv::Mat image16(img_raw.h, img_raw.w, CV_16U, img_raw.ptr);
        cv::Mat gray8;;
        image16.convertTo(gray8, CV_8U, 1.0 / 256.0);

        // Requires intermediate cbdetect::Params struct
        std::vector<cbdetect::Board> boards;
        cbdetect::Corner corners;// will be populated for this image
        cbdetect::find_corners(gray8, corners, *this->cb_params);
        cbdetect::boards_from_corners(gray8, corners, boards, *this->cb_params);// Does the filtering for us

        if (!boards.empty()) {
            ccd_good = CalibCornerData(corners, boards);// Assuming you want to use the first board
                                                        //        ccd_bad = CalibCornerData // Nothing is rejected?
        }
    }

    void OpenCVCheckerboardParams::process(basalt::ManagedImage<uint16_t> &img_raw, basalt::CalibCornerData &ccd_good, basalt::CalibCornerData &ccd_bad) {
        cv::Mat image16(img_raw.h, img_raw.w, CV_16U, img_raw.ptr);
        cv::Mat gray8;
        image16.convertTo(gray8, CV_8U, 1.0 / 256.0);

        std::vector<cv::Point2f> corners;

        bool patternfound = findChessboardCornersSB(gray8, this->pattern_size, corners, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);

        if (patternfound) {
            // cornerSubPix(gray8, corners, cv::Size(5,5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40, 0.001));

            float diff = corners[0].x - corners[this->pattern_size.width - 1].x;

            // We make a heavy assumption about the relative orientation of the camera and the chessboard.
            // Specifically, we assume that the first corner detected is always at the leftmost part of the row,
            // and the last corner of the row is always at the rightmost part. This assumption holds true as long as
            // the camera and the chessboard keep a roughly consistent relative orientation. However, in cases where
            // the orientation varies significantly, this logic may fail to correctly determine the orientation of the corners.
            //
            // A better approach would be to use an asymmetric chessboard pattern for calibration. An asymmetric
            // chessboard has different numbers of squares in its rows and columns, making it easier for the
            // corner detection algorithm to correctly orient the detected pattern regardless of the orientation of the chessboard.
            //
            // If the first corner's x-coordinate is larger than the last corner's x-coordinate,
            // the array of corners is reversed to ensure correct orientation.
            if (diff > 0) {
                std::reverse(corners.begin(), corners.end());
            }

            ccd_good = CalibCornerData(corners);
        } else {
            ccd_bad = CalibCornerData(corners);
        }
    }



    Calibrator::Calibrator(const std::shared_ptr<RosbagDataset> &dataset) {
        const fs::path temp = dataset->get_file_path();
        this->cache_path = temp.parent_path() / "calib-cam_detected_corners.cereal";
        this->dataset = dataset;
    }


    void Calibrator::detectCorners(const std::shared_ptr<CalibParams> &params) {
        if (this->loadCache()) {
            return;
        } else {
            spdlog::trace("No cached corners found, running corner detection");

            this->dataset->calib_corners.clear();
            this->dataset->calib_corners_rejected.clear();

            tbb::parallel_for(
                    tbb::blocked_range<size_t>(0, this->dataset->get_image_timestamps().size()),
                    [&](const tbb::blocked_range<size_t> &r) {
                        for (size_t j = r.begin(); j != r.end(); ++j) {
                            int64_t timestamp_ns = this->dataset->get_image_timestamps()[j];
                            const std::vector<ImageData> &img_vec = this->dataset->get_image_data(timestamp_ns);

                            for (size_t i = 0; i < img_vec.size(); i++) {
                                if (img_vec[i].img.get()) {
                                    CalibCornerData ccd_good;
                                    CalibCornerData ccd_bad;

                                    params->process(*img_vec[i].img, ccd_good, ccd_bad);
                                    spdlog::debug("image ({},{})  detected {} corners ({} rejected)",
                                                  timestamp_ns, i, ccd_good.corners.size(),
                                                  ccd_bad.corners.size());

                                    TimeCamId tcid(timestamp_ns, i);

                                    ccd_good.seq = j;
                                    ccd_bad.seq = j;

                                    this->dataset->calib_corners.emplace(tcid, ccd_good);
                                    this->dataset->calib_corners_rejected.emplace(tcid, ccd_bad);
                                }
                            }
                        }
                    });

            spdlog::debug("Successfully detected corners");
            this->saveCache();
        }
    };
}// namespace basalt