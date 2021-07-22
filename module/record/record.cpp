#include "record.hpp"

#include <iostream>

namespace Record_mode {
Record::Record() {}
Record::Record(const std::string path_input, String path_in, cv::Size size) {
  cv::FileStorage record(path_input, cv::FileStorage::READ);

  record["RECORD_SWITCH"] >> switch_r;
  record["PATH_"] >> Path_H;
  record["FPS_RECORD"] >> fps_r;
  record["RECORD_MODE"] >> mode_set;
  record["VISION_ELECTRONIC_LOCK"] >> Priority;
  palce_change = path_in;
  size_ = size;
}
void Record::RecordIng(cv::Mat src_img_r) {
  if (mode_set == 1) {
    writer << src_img_r;
  }
  if (mode_set == 2) {
    if (cv::waitKey(1) == 's') {
      n += 1;
      cv::imwrite(fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/" + std::to_string(n) + ".jpg"), src_img_r);
    }
  }
}

void Record::Change_Place(const String path_) {
  palce_change = path_;
  if (switch_r == 1) {
    writer.open(palce_change, fourcc_, fps_r, size_, true);
  }
  if (writer.isOpened()) {
    fmt::print("ISRECORDING\n");
  } else {
    fmt::print("UNABLE TO RECORD\n");
  }
}
Record::~Record() {}

}  // namespace Record_mode
