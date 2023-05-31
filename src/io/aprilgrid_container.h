#include <memory>

#include "io/files_container.h"
#include "calibration/aprilgrid.h"

class AprilGridContainer : public FilesContainer<AprilGrid> {
public:
  std::shared_ptr<AprilGrid> &operator[](int index) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= files_list.size()) {
      throw std::out_of_range(std::string("index: ") + std::to_string(index));
    }
    auto itr = files_list.begin();
    std::advance(itr, index);
    return *itr;
  }

  void addFiles(std::vector<std::string> const &files) override {
    std::lock_guard<std::mutex> lock(mtx);

    for (auto &&file: files) {
      try {
        if (std::find_if(files_list.begin(), files_list.end(),
                         [file](const std::shared_ptr<AprilGrid> &r) { return r.get()->get_file_path() == file; }) !=
            files_list.end()) {
          throw std::runtime_error(tmpstringstream() << "April Grid \"" << file << "\" is already loaded");
        }
        std::shared_ptr<AprilGrid> file_ptr = std::make_shared<AprilGrid>(file);
        files_list.emplace_back(file_ptr);
      }
      catch (const std::exception &e) {
        last_error_msg += e.what();
        last_error_msg += "\n";
      }
    }
  }
};