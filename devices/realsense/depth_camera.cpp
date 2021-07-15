/*
 * @Author: joyce
 * @Date: 2021-05-27 21:45:36
 * @LastEditTime: 2021-07-15 10:48:20
 * @LastEditors: Please set LastEditors
 * @Description:: 
 */

#include "depth_camera.hpp"

namespace depth_capture {

RM_DepthCapture::RM_DepthCapture() {}

RM_DepthCapture::~RM_DepthCapture() {}

/*__获得深度距离信息__*/
float RM_DepthCapture::getDistance(rs2::frameset _frameset) {
  rs2::depth_frame depth_frame = _frameset.get_depth_frame();
  float            width       = depth_frame.get_width();
  float            height      = depth_frame.get_height();
  distance_                    = depth_frame.get_distance(width / 2, height / 2) * 100;  // cm
  return distance_;
}

/*__获得彩色图__*/
cv::Mat RM_DepthCapture::getColorImage(rs2::pipeline _pipeline, rs2::frameset _frameset) {
  rs2::video_frame video_src = _frameset.get_color_frame();
  color_img_ =
    cv::Mat(cv::Size(video_src.as<rs2::video_frame>().get_width(),
                     video_src.as<rs2::video_frame>().get_height()),
                     CV_8UC3, reinterpret_cast<void*> video_src.get_data(), cv::Mat::AUTO_STEP);
  cv::cvtColor(color_img_, color_img_, cv::COLOR_BGR2RGB);
  return color_img_;
}

/*__获得彩色深度图__*/
cv::Mat RM_DepthCapture::getDepthImage(rs2::pipeline _pipeline, rs2::frameset _frameset) {
  rs2::depth_frame depth_frame_temp     = _frameset.get_depth_frame();
  rs2::frame       depth_frame_colormap = depth_frame_temp.apply_filter(color_map_);
  depth_img_                            =
  cv::Mat(cv::Size(depth_frame_colormap.as<rs2::video_frame>().get_width(),
                   depth_frame_colormap.as<rs2::video_frame>().get_height()),
                   CV_8UC3, reinterpret_cast<void*>depth_frame_colormap.get_data(), cv::Mat::AUTO_STEP);
  return depth_img_;
}

/*__深度图对齐到彩色图__*/
cv::Mat RM_DepthCapture::getAlignedToColorFrame(rs2::pipeline _pipeline, rs2::frameset _frameset) {
  config_.enable_stream(RS2_STREAM_COLOR);
  config_.enable_stream(RS2_STREAM_DEPTH);
  rs2::align align_to_color(RS2_STREAM_COLOR);

  _frameset            = align_to_color.process(_frameset);
  auto depth           = _frameset.get_depth_frame();
  auto colorized_depth = color_map_.colorize(depth);
  aligned_to_color_frame_ =
  cv::Mat(cv::Size(colorized_depth.as<rs2::video_frame>().get_width(),
                   colorized_depth.as<rs2::video_frame>().get_height()),
                   CV_8UC3, reinterpret_cast<void*> colorized_depth.get_data(), cv::Mat::AUTO_STEP);

  return aligned_to_color_frame_;
}

/*__彩色图对齐到深度图__*/
cv::Mat RM_DepthCapture::getAlignedToDepthFrame(rs2::pipeline _pipeline, rs2::frameset _frameset) {
  config_.enable_stream(RS2_STREAM_COLOR);
  config_.enable_stream(RS2_STREAM_DEPTH);
  rs2::align align_to_color(RS2_STREAM_DEPTH);

  _frameset            = align_to_color.process(_frameset);
  auto depth           = _frameset.get_depth_frame();
  auto colorized_depth = color_map_.colorize(depth);
  aligned_to_depth_frame_ =
  cv::Mat(cv::Size(colorized_depth.as<rs2::video_frame>().get_width(),
                   colorized_depth.as<rs2::video_frame>().get_height()),
                   CV_8UC3, reinterpret_cast<void*> colorized_depth.get_data(), cv::Mat::AUTO_STEP);

  return aligned_to_depth_frame_;
}
}  // namespace depth_capture
