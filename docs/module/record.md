# RM_Record 使用手册


## 一、使用说明

---

### 步骤一：头文件说明

- 包含头文件`record.hpp`
- 包含的源文件`record.cpp`
### 步骤二：实例化对象

实例化对象时调用构造函数：`Record(std::string record_path_, std::string path_in, cv::Size size)`

参数解释:
- `path_in` 为 `保存路径的变量参数` 的路径地址
- `record_path_`为`xml文件`
- `size` 为 `传入的图像大小`
### 步骤三：函数说明

在录制模式下调用这个函数即可

  ```cpp
visionJudge(const cv::Mat input_img, int judge, int current_mode);
RecordIng(cv::Mat img_)
  ```
  参数解释：
  |      参数名         |           参数解释             |
  | ------------------ | -----------------------------|
  | `input_img`         | 相机输入的图像，如：`src_img_`   |
  | `judge`    |判断是否使用录制开关              |
|`current_mode`| 读取当前模式|

## 二、自定义函数及参数说明

---

### 相机录制函数
第二个录制函数直接调用接口

  ```cpp
    /**
   * @brief  传入图像设置
   * @param input_img 传入图像
   * @param judge 是否录制判断
   * @param current_mode 当前模式输入
   */
  void            visionJudge(const cv::Mat input_img, int judge, int current_mode);
 ```
 ```cpp
   /**
  * @brief 录制函数
  * @param img_ 传入图像
  * 
  */
  void RecordIng(cv::Mat img_);

  ```

```cpp
  cv::VideoWriter vw_image_;  //录制对象说明
  int             path_ = 0;  // 路径
  ModeSet         mode_;
  int             last_mode_;  // 记录上次串口模式
```

ps:本模块功能还在改进完善当中,在赛场并未使用