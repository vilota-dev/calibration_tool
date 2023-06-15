#pragma once

#include "spdlog/spdlog.h"
#include <basalt/utils/sophus_utils.hpp>
#include <cereal/archives/json.hpp>

#include <fstream>

namespace basalt {
    struct AprilGrid {
        AprilGrid(int tagCols, int tagRows, double tagSize, double tagSpacing, int lowId, const std::string& tagFamily) {
            this->tagCols = tagCols;
            this->tagRows = tagRows;
            this->tagSize = tagSize;
            this->tagSpacing = tagSpacing;
            this->lowId = lowId;
            this->tagFamily = tagFamily;
            this->file_path = "";
            this->name = "AprilGrid_" + std::to_string(tagCols) + "x" + std::to_string(tagRows) + "_" +
                         std::to_string(tagSize) + "_" + std::to_string(tagSpacing) + "_" + std::to_string(lowId) + "_" +
                         tagFamily;

            double x_corner_offsets[4] = {0, tagSize, tagSize, 0};
            double y_corner_offsets[4] = {0, 0, tagSize, tagSize};

            aprilgrid_corner_pos_3d.resize(tagCols * tagRows * 4);

            for (int y = 0; y < tagRows; y++) {
                for (int x = 0; x < tagCols; x++) {
                    int tag_id = tagCols * y + x;
                    double x_offset = x * tagSize * (1 + tagSpacing);
                    double y_offset = y * tagSize * (1 + tagSpacing);

                    for (int i = 0; i < 4; i++) {
                        int corner_id = (tag_id << 2) + i;

                        Eigen::Vector4d &pos_3d = aprilgrid_corner_pos_3d[corner_id];

                        pos_3d[0] = x_offset + x_corner_offsets[i];
                        pos_3d[1] = y_offset + y_corner_offsets[i];
                        pos_3d[2] = 0;
                        pos_3d[3] = 1;
                    }
                }
            }

            int num_vign_points = 5;
            int num_blocks = tagCols * tagRows * 2;

            aprilgrid_vignette_pos_3d.resize((num_blocks + tagCols + tagRows) *
                                             num_vign_points);

            for (int k = 0; k < num_vign_points; k++) {
                for (int i = 0; i < tagCols * tagRows; i++) {
                    // const Eigen::Vector3d p0 = aprilgrid_corner_pos_3d[4 * i + 0];
                    const Eigen::Vector4d p1 = aprilgrid_corner_pos_3d[4 * i + 1];
                    const Eigen::Vector4d p2 = aprilgrid_corner_pos_3d[4 * i + 2];
                    const Eigen::Vector4d p3 = aprilgrid_corner_pos_3d[4 * i + 3];

                    double coeff = double(k + 1) / double(num_vign_points + 1);

                    aprilgrid_vignette_pos_3d[k * num_blocks + 2 * i + 0] =
                            (p1 + coeff * (p2 - p1));
                    aprilgrid_vignette_pos_3d[k * num_blocks + 2 * i + 1] =
                            (p2 + coeff * (p3 - p2));

                    aprilgrid_vignette_pos_3d[k * num_blocks + 2 * i + 0][0] +=
                            tagSize * tagSpacing / 2;
                    aprilgrid_vignette_pos_3d[k * num_blocks + 2 * i + 1][1] +=
                            tagSize * tagSpacing / 2;
                }
            }

            size_t curr_idx = num_blocks * num_vign_points;

            for (int k = 0; k < num_vign_points; k++) {
                for (int i = 0; i < tagCols; i++) {
                    const Eigen::Vector4d p0 = aprilgrid_corner_pos_3d[4 * i + 0];
                    const Eigen::Vector4d p1 = aprilgrid_corner_pos_3d[4 * i + 1];

                    double coeff = double(k + 1) / double(num_vign_points + 1);

                    aprilgrid_vignette_pos_3d[curr_idx + k * tagCols + i] =
                            (p0 + coeff * (p1 - p0));

                    aprilgrid_vignette_pos_3d[curr_idx + k * tagCols + i][1] -=
                            tagSize * tagSpacing / 2;
                }
            }

            curr_idx += tagCols * num_vign_points;

            for (int k = 0; k < num_vign_points; k++) {
                for (int i = 0; i < tagRows; i++) {
                    const Eigen::Vector4d p0 = aprilgrid_corner_pos_3d[4 * i * tagCols + 0];
                    const Eigen::Vector4d p3 = aprilgrid_corner_pos_3d[4 * i * tagCols + 3];

                    double coeff = double(k + 1) / double(num_vign_points + 1);

                    aprilgrid_vignette_pos_3d[curr_idx + k * tagRows + i] =
                            (p0 + coeff * (p3 - p0));

                    aprilgrid_vignette_pos_3d[curr_idx + k * tagRows + i][0] -=
                            tagSize * tagSpacing / 2;
                }
            }
        }

        AprilGrid(const std::string &config_path) {
            std::ifstream is(config_path);
            if (is.is_open()) {
                cereal::JSONInputArchive ar(is);
                ar(cereal::make_nvp("tagCols", tagCols));
                ar(cereal::make_nvp("tagRows", tagRows));
                ar(cereal::make_nvp("tagSize", tagSize));
                ar(cereal::make_nvp("tagSpacing", tagSpacing));
                ar(cereal::make_nvp("tagFamily", tagFamily));
                ar(cereal::make_nvp("lowId", lowId));
            } else {
                spdlog::error("Could not open aprilgrid configuration: {}", config_path);
                std::abort();
            }

            AprilGrid(tagCols, tagRows, tagSize, tagSpacing, lowId, tagFamily);
            this->file_path = config_path;
            this->name = this->file_path.filename().string();
        }

        Eigen::aligned_vector<Eigen::Vector4d> aprilgrid_corner_pos_3d;
        Eigen::aligned_vector<Eigen::Vector4d> aprilgrid_vignette_pos_3d;

        inline const std::filesystem::path &get_file_path() const { return file_path; }
        inline const std::string &get_name() const { return name; }
        inline int getTagCols() const { return tagCols; }
        inline int getTagRows() const { return tagRows; }
        inline double getTagSize() const { return tagSize; }
        inline double getTagSpacing() const { return tagSpacing; }
        inline int getLowId() const { return lowId; }
        inline std::string getTagFamily() const { return tagFamily; }

    private:
        std::filesystem::path file_path;
        std::string name;
        int tagCols;      // number of apriltags
        int tagRows;      // number of apriltags
        double tagSize;   // size of apriltag, edge to edge [m]
        double tagSpacing;// ratio of space between tags to tagSize
        std::string tagFamily;
        double lowId;
    };

    typedef std::shared_ptr<AprilGrid> AprilGridPtr;

}// namespace basalt