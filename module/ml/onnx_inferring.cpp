#include "onnx_inferring.hpp"
#include <string>

namespace onnx_inferring {

Number_Param::Number_Param(std::string path_input) {
  cv::FileStorage model_ocr(path_input, cv::FileStorage::READ);
  model_ocr["H_MIN_NUM"] >> h_min_num;
  model_ocr["H_MAX_NUM"] >> h_max_num;
  model_ocr["S_MIN_NUM"] >> s_min_num;
  model_ocr["S_MAX_NUM"] >> s_max_num;
  model_ocr["V_MIN_NUM"] >> v_min_num;
  model_ocr["V_MAX_NUM"] >> v_max_num;
  model_ocr["CONFIDENT"] >> Confident;
  model_ocr["KERNEL_SIZE"] >> kennerl_size;

  model_ocr["NUMBER_SWITCH"] >> switch_number;
}

}  // namespace onnx_inferring
