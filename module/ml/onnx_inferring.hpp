#pragma once

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

namespace onnx_inferring {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "onnx_inferring");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "onnx_inferring");

class onnx_model {
 public:
  inline onnx_model(const char*     _onnx_model_path,
                    const cv::Size& _input_size);
    
  inline int inferring(const cv::Mat& _input,
                       const int      _median_blur_kernel_size,
                       const float    _probability_threshold);
    
 protected:
  inline void load(const char* _onnx_model_path);

  inline void layers(void);
    
private:
  cv::dnn::Net opencv_net_;
  const char*  model_path_;
  
  cv::Size     input_size_;
  cv::Mat      tmp_;
};

}