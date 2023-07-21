#pragma once

#include "spdlog/spdlog.h"
#include <basalt/utils/sophus_utils.hpp>
#include <cereal/archives/json.hpp>

#include <fstream>

namespace basalt {
    struct AprilGrid {
        
        // intended for immediate filling with serialisation archive
        AprilGrid() {};

        AprilGrid(const std::string &config_path) {
            std::ifstream is(config_path);
            if (is.is_open()) {
                cereal::JSONInputArchive ar(is);
                load(ar); // https://stackoverflow.com/questions/33726072/how-to-serialize-a-json-object-without-enclosing-it-in-a-sub-object-using-cereal
            } else {
                spdlog::error("Could not open aprilgrid configuration: {}", config_path);
                std::abort();
            }

            // AprilGrid(tagCols, tagRows, tagSize, tagSpacing, lowId, tagFamily);
            // this->file_path = config_path;
            this->name = config_path;
        }

        // inline const std::filesystem::path &get_file_path() const { return file_path; }
        inline const std::string &get_name() const { return name; }
        inline int getTagCols() const { return tagCols; }
        inline int getTagRows() const { return tagRows; }
        inline double getTagSize() const { return tagSize; }
        inline double getTagSpacing() const { return tagSpacing; }
        inline int getLowId() const { return lowId; }
        inline std::string getTagFamily() const { return tagFamily; }


        // internal split/load function
        // https://uscilab.github.io/cereal/serialization_functions.html
        template<class Archive>
        void save(Archive &ar) const{

            ar(cereal::make_nvp("targetType", std::string("aprilgrid"))); // convention https://github.com/ethz-asl/kalibr/wiki/calibration-targets
            ar(cereal::make_nvp("tagCols", tagCols));
            ar(cereal::make_nvp("tagRows", tagRows));
            ar(cereal::make_nvp("tagSize", tagSize));
            ar(cereal::make_nvp("tagSpacing", tagSpacing));
            ar(cereal::make_nvp("tagFamily", tagFamily));
            ar(cereal::make_nvp("lowId", lowId));
        }

        template<class Archive>
        void load(Archive &ar){
            // check for target_type
            std::string targetType; 
            ar(cereal::make_nvp("targetType", targetType));
            if (targetType != "aprilgrid")
                throw std::runtime_error("trying to serialise a AprilGrid object with non aprilgrid targetType");

            ar(cereal::make_nvp("tagCols", tagCols));
            ar(cereal::make_nvp("tagRows", tagRows));
            ar(cereal::make_nvp("tagSize", tagSize));
            ar(cereal::make_nvp("tagSpacing", tagSpacing));
            ar(cereal::make_nvp("tagFamily", tagFamily));
            ar(cereal::make_nvp("lowId", lowId));
        }
        
        // compare equal by serialised format
        bool operator==(const AprilGrid& rhs) const{
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
                spdlog::info("AprilGrid param matched: \n{}", os_lhs.str());
                return true;
            }else {
                spdlog::info("AprilGrid param mismatched \n{}\n---and---\n{}", os_lhs.str(), os_rhs.str());
                return false;
            }
        }

        bool operator!=(const AprilGrid& rhs) const{
            return !(*this==rhs);
        }


    private:
        // std::filesystem::path file_path;
        std::string name;

        // below are the serialisable params
        int tagCols;      // number of apriltags
        int tagRows;      // number of apriltags
        double tagSize;   // size of apriltag, edge to edge [m]
        double tagSpacing;// ratio of space between tags to tagSize
        std::string tagFamily;
        double lowId;
    };

    typedef std::shared_ptr<AprilGrid> AprilGridPtr;

}// namespace basalt