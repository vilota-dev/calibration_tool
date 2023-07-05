#pragma once
#include "calibration/aprilgrid.hpp"
#include "io/dataset_io.h"
#include "utils/utils.hpp"

#include <mutex>
#include <list>
#include <string>
#include <tuple>
#include <fstream>
#include <memory>

class AprilGridContainer {
public:
  size_t size() {
    std::lock_guard<std::mutex> lock(mtx);
    return files_list.size();
  }

  basalt::AprilGridPtr &operator[](int index) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= files_list.size()) {
      throw std::out_of_range(std::string("index: ") + std::to_string(index));
    }
    auto itr = files_list.begin();
    std::advance(itr, index);
    return *itr;
  }

  void addFiles(std::vector<basalt::AprilGridPtr> const &files) {
    std::lock_guard<std::mutex> lock(mtx);

    for (auto &&file: files) {
      try {
        if (std::find_if(files_list.begin(), files_list.end(),
                         [file](const basalt::AprilGridPtr &r) { return r->get_name() == file->get_name(); }) !=
            files_list.end()) {
          throw std::runtime_error(tmpstringstream() << "April Grid \"" << file << "\" is already loaded");
        }
        files_list.emplace_back(file);
        spdlog::debug("Successfully added April Grid file: {}", file->get_name());
      }
      catch (const std::exception &e) {
        spdlog::error("{}", e.what());
        spdlog::warn("Please fix the file and try again");
      }
    }
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

protected:
  std::mutex mtx;
  std::list<basalt::AprilGridPtr> files_list;
};