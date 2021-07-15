/*
 * @Author: your name
 * @Date: 2021-07-13 16:56:33
 * @LastEditTime: 2021-07-15 10:51:37
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /WolfVision/module/ml/yolov3_dnn.h
 */

#pragma once

#include <algorithm>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <string>
#include <vector>
#include "devices/realsense/depth_camera.hpp"
namespace yolov3_dnn {
class RM_Engineering {
 public:
  std::vector<cv::Point>   center_x_y_;           // 中心点点集
  float                    confThreshold_ = 0.5;  // 置信阈值
  float                    nmsThreshold_  = 0.4;  // 非最大抑制阈值
  int                      inpWidth_      = 416;  // 网络输入图像宽度
  int                      inpHeight_     = 416;  // 网络输入图像高度
  std::vector<cv::Mat>     outs_;                 // 获取输出层的输出
  std::vector<std::string> classes_;              // 类名标签合集
  std::string              modelConfiguration_;   // 模型cfg设置文件路径
  std::string              modelWeights_;         // 模型路径
  cv::dnn::Net             net_;                  // 网络

 public:
  RM_Engineering();
  ~RM_Engineering();

  /*加载类名称文件*/
  std::vector<std::string> setClassNames(std::string _classesFile);

  /*加载网络*/
  cv::dnn::Net setNet(std::string _modelConfiguration, std::string _modelWeights);

  /*获取输出层的名称*/
  std::vector<cv::String> getOutputsNames(const cv::dnn::Net& _net);

  /*获取输出层的输出*/
  std::vector<cv::Mat> getOutputsLayer(cv::dnn::Net& _net, cv::Mat& _frame);
  std::vector<cv::Mat> getOutputsLayer(cv::dnn::Net& _net, cv::Mat& _frame, int _inpWidth, int _inpHeight);

  /*矩形框筛选*/
  std::vector<cv::Point> postprocess(cv::Mat& _frame, std::vector<cv::Mat>& _outs, int _min_distance, std::vector<std::string> _classes);
  std::vector<cv::Point> postprocess(cv::Mat& _frame, std::vector<cv::Mat>& _outs, int _min_distance, std::vector<std::string> _classes, float _confThreshold, float _nmsThreshold);

  /*画矩形框*/
  void drawPred(int _classId, float _conf, int _left, int _top, int _right, int _bottom, cv::Mat& _frame, int _min_distance, std::vector<std::string> _classes);
};

}  // namespace yolov3_dnn
