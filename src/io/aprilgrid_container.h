#pragma once
#include "calibration/aprilgrid.h"
#include "dataset_io.h"
#include "utils/utils.h"

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

  std::shared_ptr<basalt::AprilGrid> &operator[](int index) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= files_list.size()) {
      throw std::out_of_range(std::string("index: ") + std::to_string(index));
    }
    auto itr = files_list.begin();
    std::advance(itr, index);
    return *itr;
  }

  void addFiles(std::vector<std::string> const &files) {
    std::lock_guard<std::mutex> lock(mtx);

    for (auto &&file: files) {
      try {
        if (std::find_if(files_list.begin(), files_list.end(),
                         [file](const std::shared_ptr<basalt::AprilGrid> &r) { return r.get()->get_file_path() == file; }) !=
            files_list.end()) {
          throw std::runtime_error(tmpstringstream() << "April Grid \"" << file << "\" is already loaded");
        }
        std::shared_ptr<basalt::AprilGrid> file_ptr = std::make_shared<basalt::AprilGrid>(file);
        files_list.emplace_back(file_ptr);
      }
      catch (const std::exception &e) {
        last_error_msg += e.what();
        last_error_msg += "\n";
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
  std::list<std::shared_ptr<basalt::AprilGrid>> files_list;
  std::string last_error_msg;
};