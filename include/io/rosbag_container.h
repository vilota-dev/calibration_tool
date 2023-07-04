#pragma once

#include "dataset_io.h"
#include "utils/utils.hpp"
#include "calibration/aprilgrid.hpp"

#include <mutex>
#include <list>
#include <string>
#include <tuple>
#include <fstream>

class RosbagContainer {
public:
  size_t size() {
    std::lock_guard<std::mutex> lock(mtx);
    return files_list.size();
  }

  int remove_file(int index) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= files_list.size())
      throw std::out_of_range(std::string("index: ") + std::to_string(index));

    auto it_to_remove = files_list.begin();
    std::advance(it_to_remove, index);
    auto it = files_list.erase(it_to_remove);
    return std::distance(files_list.begin(), it);
  }

  std::shared_ptr<basalt::RosbagDataset> &operator[](int index) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= files_list.size()) {
      throw std::out_of_range(std::string("index: ") + std::to_string(index));
    }
    auto itr = files_list.begin();
    std::advance(itr, index);
    return *itr;
  }

  void addFiles(std::vector<std::string> const &files)  {
    std::lock_guard<std::mutex> lock(mtx);

    for (auto &&file: files) {
      try {
        if (std::find_if(files_list.begin(), files_list.end(),
                         [file](const std::shared_ptr<basalt::RosbagDataset> r) { return r->get_file_path() == file; }) !=
            files_list.end()) {
          throw std::runtime_error(tmpstringstream() << "ROS .bag \"" << file << "\" is already loaded");
        }
        std::shared_ptr<basalt::RosbagDataset> file_ptr = std::make_shared<basalt::RosbagDataset>(file);
        files_list.emplace_back(file_ptr);
        spdlog::debug("Successfully added ROS .bag file: {}", file);
      }
      catch (const std::exception &e) {
        spdlog::warn("{}", e.what());
        spdlog::warn("Please fix the file and try again");
      }
    }
  }

protected:
  std::mutex mtx;
  std::list<std::shared_ptr<basalt::RosbagDataset>> files_list;
};