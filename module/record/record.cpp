#include "record.hpp"

#include <iostream>

namespace Record_mode {
Record::Record() {}
Record::Record(std::string path_input, String path_in, cv::Size size) {
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

void Record::Change_Place(String path_) {
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

void Record::Vision_judge(const cv::Mat src_input,  int judge_ , int current_mode) {
// 视觉部分
  if (Priority) {
    if (judge_ == 's') {
      vidion_up = true;
      if (!Recording_flag) {
        Rmode_current = S5;
        Change_Place(fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/Record" + std::to_string(n++) + ".avi"));
        Rmode_last     = Rmode_current;
        Recording_flag = true;
      }
    }
    if (judge_ == 'e') {
      vidion_up      = false;
      Rmode_current  = S1;
      Rmode_last     = Rmode_current;
      Recording_flag = false;
    }
    if (vidion_up) {
      RecordIng(src_input);
    }
  }
  // 串口部分
  if (current_mode == S5) {
    RecordIng(src_input);
    if (!Recording_flag) {
      Change_Place(fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/Record" + std::to_string(n++) + ".avi"));
      Rmode_last     = current_mode;
      Recording_flag = true;
    }
  }
  if (current_mode != S5 && !vidion_up) {
    Recording_flag = false;
  }
  if (Rmode_last != S5 && !Recording_flag) {
    writer.release();
  }
}
Record::~Record() {}

}  // namespace Record_mode
