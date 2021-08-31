# Onnx_Inferring 使用手册
## 一、使用说明

### 步骤一：头文件说明

- 包含头文件 `onxx_inferring.hpp`
- 包含的源文件 `onnx_inferring.cpp`
- 包含的模型 `minist-8.onnx`
### 步骤二：实例化对象
实例化对象时调用构造函数：`inline explicit model(std::string onnx_model_path, const cv::Size& input_size = cv::Size(28, 28))`

参数解释:
- `onnx_model_path` 为 `模型路径` 的路径地址
- `input_size` 为 `传入图像的大小 (28 x 28)`
### 步骤三：函数说明
在哨兵模式下调用这个函数。
```cpp
int inferring(const cv::Mat& hsv_input, const int median_blur_kernel_size = 3, float probability_threshold = 0, cv::Mat image_input = cv::Mat::zeros(cv::Size(255, 0), CV_8UC3));
```
参数解释：
|      参数名         |           参数解释             |
| ------------------ | -----------------------------|
| `hsv_input`         | 传入的二值化图像  |
| `median_blur_kernel_size`    |中值滤波 size 大小默认设置为 3          |
| `probability_threshold`    | 二值化大小设置          |
| `image_input`    | 原图传入     |
类内调用函数:
```cpp
model(std::string onnx_model_path, const cv::Size& input_size = cv::Size(28, 28));  // 初始化构造函数加载模型
```
参数解释：
|      参数名         |           参数解释             |
| ------------------ | -----------------------------|
| `onnx_model_path`         | 模型路径 `./module/ml/mnist-8.onnx`   |
| `input_size`    |图像的大小(默认更改为 28 x 28)              |
## 二、自定义函数说明
### 模型调用函数
```cpp
inline int inferring(const cv::Mat& hsv_input, const int median_blur_kernel_size = 3, float probability_threshold = 0, cv::Mat image_input = cv::Mat::zeros(cv::Size(255, 0), CV_8UC3));
```
设计思路:  
通过输入的原图在识别到装甲板的后在所选的区域转换程hsv图像进行识别，其中通过内核默认值为3的中值滤波对图像进行处理，提高识别的准确性。

### 模型加载
```cpp
inline void load(std::string onnx_model_path);
```
设计思路:  
输入训练好的模型路径并且加载对传入的装甲板对象进行识别