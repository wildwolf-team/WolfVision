#include "onnx_inferring.hpp"

namespace onnx_inferring {

inline onnx_model::onnx_model(const char*     _onnx_model_path,
                              const cv::Size& _input_size = cv::Size(28, 28)) {
  load(_onnx_model_path);
  layers();

  input_size_ = _input_size;
}
    
inline int onnx_model::inferring(const cv::Mat& _input,
                                 const int      _median_blur_kernel_size = 3,
                                 const float    _probability_threshold   = 0) {
  cv::resize(_input, tmp_, input_size_);
  if (tmp_.channels() != 1) {
      cv::cvtColor(tmp_, tmp_, cv::COLOR_BGR2GRAY);
  }
  if (_median_blur_kernel_size != 0) {
      cv::medianBlur(tmp_, tmp_, _median_blur_kernel_size);
  }
  cv::threshold(tmp_, tmp_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  opencv_net_.setInput(cv::dnn::blobFromImage(tmp_));
  
  float max_probability     { 0 };
  int   max_probability_idx { 0 };
  int   i                   { -1 };

  fmt::print("[{}] Info, inffering results:", idntifier_green);

  opencv_net_.forward().forEach<float>(
    [&_probability_threshold, &max_probability, &max_probability_idx, &i]
    (float &data, [[maybe_unused]] const int * position) -> void {
      if (++i) {
          if (data > max_probability) {
              max_probability     = data;
              max_probability_idx = i;
          }
      } else {
          if (data > _probability_threshold) {
              max_probability = data;
          } else {
              max_probability = _probability_threshold;
              max_probability_idx = -1;
          }
      }
  
      fmt::print(" {}", data);
  });

  fmt::print("\n");
  
  return max_probability_idx;
}
    

inline void onnx_model::load(const char* _onnx_model_path) {
  fmt::print("[{}] ONNX inferring using opencv: {}\n", idntifier_green, cv::getVersionString());
    
  model_path_ = _onnx_model_path;
  opencv_net_ = cv::dnn::readNetFromONNX(model_path_);
  
  if (!opencv_net_.empty()) {
    fmt::print("[{}] Load model success: {}\n", idntifier_green, model_path_);
  } else {
    fmt::print("[{}] Load model failed: {}\n", idntifier_red, model_path_);
  }
        
#if __has_include(<opencv2/gpu/gpu.hpp>)
  opencv_net_.setPreferableBackend(cv::dnn::DNN_TARGET_CUDA);
#else
  opencv_net_.setPreferableBackend(cv::dnn::DNN_TARGET_CPU);
#endif
}

inline void onnx_model::layers(void) {
  if (opencv_net_.empty()) {
    fmt::print("[{}] Error, phrasing model layers failed\n", idntifier_red);
  }
}

}
