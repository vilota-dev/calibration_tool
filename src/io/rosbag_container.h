#include "io/files_container.h"

class RosbagContainer : public FilesContainer<RosbagDataset> {
public:
  std::shared_ptr<RosbagDataset> &operator[](int index) {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= files_list.size()) {
      throw std::out_of_range(std::string("index: ") + std::to_string(index));
    }
    auto itr = files_list.begin();
    std::advance(itr, index);
    return *itr;
  }

  void addFiles(std::vector<std::string> const &files) override {
    spdlog::trace("RosbagContainer::addFiles");
    std::lock_guard<std::mutex> lock(mtx);

    for (auto &&file: files) {
      // No error checking for now, just assume it works
      std::shared_ptr<RosbagDataset> file_ptr = std::shared_ptr<RosbagDataset>(new RosbagDataset(file));
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
};