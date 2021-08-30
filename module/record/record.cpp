/**
 * @file record.cpp
 * @author SMS (2436210442@qq.com)
 * @brief 视频保存
 * @date 2021-08-28
 *
 * @copyright Copyright (c) 2021 GUCROBOT_WOLF
 *
 */
#include "record.hpp"

#include <iostream>

namespace RecordMode {
ReCord::ReCord() {}
ReCord::ReCord(std::string path_input, std::string path_in, cv::Size size) {
    video_save_path_ = path_input;
}
ReCord::~ReCord() {}

}  // namespace RecordMode
