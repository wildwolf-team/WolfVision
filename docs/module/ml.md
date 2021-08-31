# Onnx_inferring 使用手册
## 一、使用说明

### 步骤一：头文件说明

- 包含头文件 `onxx_inferring.hpp`
- 包含的源文件 `onnx-inferring.cpp`
- 包含的模型 `minist-8.onnx`
### 步骤二：实例化对象

实例化对象时调用构造函数：`  inline explicit model(std::string onnx_model_path, const cv::Size& input_size = cv::Size(28, 28));`

参数解释:
- `onnx_model_path` 为 `模型路径` 的路径地址
- `input_size` 为 `传入图像的大小(28x28);`
### 步骤三：函数说明

在录制模式下调用这个函数即可。
```cpp
int inferring(const cv::Mat& hsv_input, const int median_blur_kernel_size = 3, float probability_threshold = 0, cv::Mat image_input = cv::Mat::zeros(cv::Size(255, 0), CV_8UC3));
```
参数解释：
|      参数名         |           参数解释             |
| ------------------ | -----------------------------|
| `hsv_input`         | 传入的二值化图像  |
| `median_blur_kernel_size`    |中值滤波size大小默认设置为3          |
| `probability_threshold`    |二值化大小设置          |
| `image_input`    |原图传入     |

类内调用函数:
```cpp
model(std::string onnx_model_path, const cv::Size& input_size = cv::Size(28, 28));  // 初始化构造函数加载模型
```
参数解释：
|      参数名         |           参数解释             |
| ------------------ | -----------------------------|
| `onnx_model_path`         | 模型路径`./module/ml/mnist-8.onnx`   |
| `input_size`    |图像的大小(默认更改为28x28)              |
## 二、自定义函数及参数说明
### 模型调用函数

```cpp

/**
* 
@brief:  Inferring input image from loaded model, return classified int digit
@param:  input, the image to classify (only 1 digit), const reference from cv::Mat
@param:  median_blur_kernel_size, define the kernel size of median blur pre-processing, default to int 5, set 0 to disable
@param:  hsv_lowerb, the lower range for hsv image, pixels inside the range equals to 1, otherwise equals to 0, default is the cv::Scalar() default
@param:  hsv_upperb, the upper range for hsv image, pixels inside the range equals to 1, otherwise equals to 0, default is the cv::Scalar() default
@param:  probability_threshold, the min probability of considerable probability to iterate, determined by the model, mnist-8.onnx has the output array from -1e5 to 1e5, default is 0
@return: max_probability_idx, the most probable digit classified from input image in int type, -1 means all the probability is out of the threahold
*/
inline int inferring(const cv::Mat& hsv_input, const int median_blur_kernel_size = 3, float probability_threshold = 0, cv::Mat image_input = cv::Mat::zeros(cv::Size(255, 0), CV_8UC3));

```
### 模型加载
```cpp
/**
@brief: Load model from onnx_model_path
@param: onnx_model_path,the path of the modle on your machine, downloadable at https://github.com/onnx/models/blob/master/vision/classification/mnist/model/mnist-8.onnx
*/
inline void load(std::string onnx_model_path);
```
