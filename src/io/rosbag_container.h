#pragma once

#include "dataset_io.h"
#include "utils/utils.h"
#include "utils/colors.h"
#include "calibration/aprilgrid.h"

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
    spdlog::trace("RosbagContainer::addFiles");
    std::lock_guard<std::mutex> lock(mtx);

    for (auto &&file: files) {
      // No error checking for now, just assume it works
      std::shared_ptr<basalt::RosbagDataset> file_ptr = std::shared_ptr<basalt::RosbagDataset>(new basalt::RosbagDataset(file));
      files_list.emplace_back(file_ptr);
//      try {
//        if (std::find_if(files_list.begin(), files_list.end(),
//                         [file](const std::shared_ptr<RosbagDataset> r) { return r->get_file_path() == file; }) !=
//            files_list.end()) {
//          throw std::runtime_error(tmpstringstream() << "ROS .bag \"" << file << "\" is already loaded");
//        }
//        std::shared_ptr<RosbagDataset> file_ptr = std::make_shared<RosbagDataset>(file);
//        files_list.emplace_back(file_ptr);
//      }
//      catch (const std::exception &e) {
//        spdlog::debug("exception {}", e.what());
//        last_error_msg += e.what();
//        last_error_msg += "\n";
//      }
    }
  }

  bool has_errors() const {
    return !last_error_msg.empty();
  }

  std::string get_last_error() {
    return last_error_msg;
  }

  void clear_errors() {
    last_error_msg.clear();
  }

protected:
  std::mutex mtx;
  std::list<std::shared_ptr<basalt::RosbagDataset>> files_list;
  std::string last_error_msg;
};