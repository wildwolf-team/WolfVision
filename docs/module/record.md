# Record 使用手册
## 一、使用说明
### 步骤一：头文件说明

- 包含头文件 ` record.hpp`
- 包含的源文件 `record.cpp`
### 步骤二：实例化对象

实例化对象时调用构造函数: `Record(std::string record_path_, std::string path_in, cv::Size size);`
参数解释:
- `path_in` 为 `保存路径的变量参数` 的路径地址
- `record_path_` 为 `xml 文件`
- `size` 为 `传入的图像大小`
### 步骤三：函数说明
在录制模式下调用这个函数即可。
```cpp
void visionRecord(const cv::Mat input_img, int judge, int current_mode);
void imgRecord(cv::Mat img_);
```
参数解释： 
|      参数名         |           参数解释             |
| ------------------ | -----------------------------|
| `input_img`         | 相机输入的图像，如: `src_img_`   |
| `judge`    |判断是否使用录制开关              |
| `current_mode`| 读取当前模式|
| `img_`| 读取相机输入图像|
## 二、自定义函数说明
### 相机录制函数
```cpp
void visionRecord(const cv::Mat input_img, int judge, int current_mode);
```
设计思路:  
根据传入的相机原图，录制开关的控制来写保存小电脑读取的视频  。

### 录制函数直接调用接口。
```cpp
void imgRecord(cv::Mat img_);
```
设计思路:  
直接调用这个函数传入相机读取到的原图像进行录制一边便在赛场上测试参数。

ps:本模块功能还在改进完善当中,在赛场并未使用。
