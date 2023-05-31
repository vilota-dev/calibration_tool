#pragma once

#include "dataset_io.h"
#include "utils/utils.h"
#include "calibration/aprilgrid.h"

#include <mutex>
#include <list>
#include <string>
#include <tuple>
#include <fstream>

using namespace basalt;

template <typename FileType>
class FilesContainer {
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

  virtual void addFiles(std::vector<std::string> const &files) = 0;

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
  std::list<std::shared_ptr<FileType>> files_list;
  std::string last_error_msg;
};