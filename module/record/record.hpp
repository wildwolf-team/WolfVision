#pragma once
#include <fmt/color.h>
#include <fmt/core.h>
#include <string>

#include <opencv2/opencv.hpp>
typedef std::string String;

namespace Record_mode {
class Record {
 public:
        Record();
explicit Record(const std::string record_path_, String path_in, cv::Size size);
        ~Record();
        int Return_switch() {return switch_r;}
        int Path_H;
        cv::VideoWriter writer;
        int             mode_set;

 private:
       int n   = 1;
        int fps_r;
        int      switch_r;
        int fourcc_ = writer.fourcc('M', 'J', 'P', 'G');
        char filename_[200];
 public :
    void RecordIng(cv::Mat Img);
};
}  // namespace Record_mode
