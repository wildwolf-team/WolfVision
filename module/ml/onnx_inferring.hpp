/*
 * @name: opencv_onnx_inferring.hpp
 * @namespace: ooi
 * @class: model
 * @brief: Load ONNX model and infer input image for classified int digit
 * @author Unbinilium
 * @version 1.0.1
 * @date 2021-05-10
 */

#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <string>
#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>

namespace onnx_inferring {

typedef struct Number_Param {
  int h_min_num;
  int h_max_num;

  int s_min_num;
  int s_max_num;

  int v_min_num;
  int v_max_num;

  int Confident;
  int kennerl_size;

  int switch_number;
  Number_Param(std::string path_in);
} NumParam;

class model {
 public:
  /**
         @brief: Init model from params
         @param: onnx_model_path, the path of the modle on your machine, downloadable at https://github.com/onnx/models/blob/master/vision/classification/mnist/model/mnist-8.onnx
         @param: input_size, define the input layer size, default to cv::Size(28, 28)
         */
  explicit model(std::string onnx_model_path, const cv::Size& input_size = cv::Size(28, 28)) {
    model::load(onnx_model_path);
    model::layers();

    this->input_size = input_size;
  }
  model(){};
  // model(std::string orc_input_path_,bool key_input);
  NumParam param_ocr = Number_Param("/home/sms/workspace_vscode/WolfV_2/WolfVision/configs/ml/onnx_inferring_config.xml");
  bool     parma_case;
  ~model(){};
  /**
         @brief:  Inferring input image from loaded model, return classified int digit
         @param:  input, the image to classify (only 1 digit), const reference from cv::Mat
         @param:  median_blur_kernel_size, define the kernel size of median blur pre-processing, default to int 5, set 0 to disable
         @param:  hsv_lowerb, the lower range for hsv image, pixels inside the range equals to 1, otherwise equals to 0, default is the cv::Scalar() default
         @param:  hsv_upperb, the upper range for hsv image, pixels inside the range equals to 1, otherwise equals to 0, default is the cv::Scalar() default
         @param:  probability_threshold, the min probability of considerable probability to iterate, determined by the model, mnist-8.onnx has the output array from -1e5 to 1e5, default is 0
         @return: max_probability_idx, the most probable digit classified from input image in int type, -1 means all the probability is out of the threahold
         */
  /*/ inline int inferring(const cv::Mat& hsv_input, const int median_blur_kernel_size = 5, const cv::Scalar& hsv_lowerb = cv::Scalar(), 
                                                 const cv::Scalar& hsv_upperb = cv::Scalar(), const float probability_threshold = 0) */
  inline int inferring(const cv::Mat& hsv_input, const int median_blur_kernel_size = 3, float probability_threshold = 0) {
    cv::resize(hsv_input, Image_output, input_size);

    cv::namedWindow("数字number");
    cv::createTrackbar("h_min_num", "数字number", &param_ocr.h_min_num, 255, NULL);
    cv::createTrackbar("s_min_num", "数字number", &param_ocr.s_min_num, 255, NULL);
    cv::createTrackbar("v_min_num", "数字number", &param_ocr.v_min_num, 255, NULL);
    // 置信度
    cv::createTrackbar("CONFIDENT", "数字number", &param_ocr.Confident, 5000, NULL);
    cv::createTrackbar("kernel_size", "数字number", &param_ocr.kennerl_size, 10, NULL);
    if (median_blur_kernel_size != 0) {
      cv::medianBlur(Image_output, Image_output, param_ocr.kennerl_size * 2 - 1);
    }
    cv::inRange(Image_output, cv::Scalar(param_ocr.h_min_num, param_ocr.s_min_num, param_ocr.v_min_num), cv::Scalar(255, 255, 255), Image_output);

    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(Image_output, Image_output, cv::MORPH_OPEN, element);
    // cv::dilate(Image_output, Image_output, cv::Mat(cv::Size(3, 3), CV_8UC1));
    // cv::bitwise_not(Image_output, Image_output);

    cv::imshow("output_img_ocr", Image_output);

    opencv_net.setInput(cv::dnn::blobFromImage(Image_output));

    float max_probability{0};
    int   max_probability_idx{0};
    int   i{-1};
    probability_threshold = param_ocr.Confident;
    opencv_net.forward().forEach<float>([&probability_threshold, &max_probability, &max_probability_idx, &i](float& data, [[maybe_unused]] const int* position) -> void {
      if (++i) {
        if (data > max_probability) {
          max_probability     = data;
          max_probability_idx = i;
        }
      } else {
        if (data > probability_threshold) {
          max_probability = data;
        } else {
          max_probability     = probability_threshold;
          max_probability_idx = -1;
        }
      }
    });

    return max_probability_idx;
  }

 protected:
  /*
         @brief: Load model from onnx_model_path
         @param: onnx_model_path, the path of the modle on your machine, downloadable at https://github.com/onnx/models/blob/master/vision/classification/mnist/model/mnist-8.onnx
         */
  inline void load(std::string onnx_model_path) {
    std::cout << "[OOI] opencv version: " << cv::getVersionString() << std::endl;

    model_path = onnx_model_path;
    opencv_net = cv::dnn::readNetFromONNX(model_path);

    if (!opencv_net.empty()) {
      std::cout << "[OOI] load model success: " << model_path << std::endl;
    } else {
      std::cout << "[OOI] load model failed: " << model_path << std::endl;
      return;
    }

#if __has_include(<opencv2/gpu/gpu.hpp>)
    opencv_net.setPreferableBackend(cv::dnn::DNN_TARGET_CUDA);
#else
    opencv_net.setPreferableBackend(cv::dnn::DNN_TARGET_CPU);
#endif
  }
  /*
         @brief: Print model layers detail from loaded model
         */
  inline void layers(void) {
    if (opencv_net.empty()) {
      std::cout << "[OOI] model is empty" << std::endl;
      return;
    }

    std::cout << "[OOI] model from " << model_path << " has layers: " << std::endl;
    for (const auto& layer_name : opencv_net.getLayerNames()) {
      std::cout << "\t\t" << layer_name << std::endl;
    }
  }

 private:
  cv::dnn::Net opencv_net;
  std::string  model_path;

  cv::Size input_size;
  cv::Mat  Image_output;
};

}  // namespace onnx_inferring

