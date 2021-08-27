# Angle_Solve 使用手册

---

## 一、使用说明

---

### 步骤一：头文件说明

- 包含头文件`basic_pnp.hpp`

| 文件名                  | 文件说明                                   |
| ---------------------- | ------------------------------------------- |
| `abstract_pnp.hpp`     | 角度结算抽象类           |
### 步骤二：实例化对象

实例化对象时调用构造函数：`explicit PnP(std::string _camera_path, std::string _pnp_config_path);`

参数解释：
- `_camera_path` 为 `相机标定文件` 的路径地址
- `_pnp_config_path` 为 `basic_pnp_config.xml` 的路径地址
  
### 步骤三：函数说明

在模式选择后调用接口函数即可,接口函数重载了12种调用方式：

  ```C++
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::Mat& _src_img, cv::RotatedRect _rect);
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::Mat& _src_img, cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height,  cv::Mat& _src_img, cv::RotatedRect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height, cv::Mat& _src_img, cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::RotatedRect _rect);            
  void PnP::solvePnP(int _ballet_speed, int _armor_type, cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height, cv::RotatedRect _rect);
  void PnP::solvePnP(int _ballet_speed, int _width, int _height, 
  cv::Rect _rect);
  void PnP::solvePnP(int _ballet_speed, int _armor_type,
  std::vector<cv::Point2f> _target_2d);
  void PnP::solvePnP(int  _ballet_speed, int _armor_type, cv::Mat&  _src_img, std::vector<cv::Point2f> _target_2d);
  void PnP::solvePnP(int _ballet_speed, int _armor_type,cv::RotatedRect _rect, int _depth)
  void PnP::solvePnP(int _ballet_speed, int _armor_type, std::vector<cv::Point2f> _target_2d, int _depth)


  ```
  参数解释：
  |      参数名         |           参数解释             |
  | ------------------ | -----------------------------|
  | `_ballet_speed`    | 子弹发射速度 |
  | `_armor_type`      | 装甲板类型 0 小装甲 1 大装甲 2 能量机关| 
  | `_src_img`         | 相机输入的图像，如：`src_img`，可删除不进行绘制 |
  | `_rect`            | 计算目标旋转矩形或者外接矩形 |
  | `_width`           | 目标实际宽度，与高度一起可替换`_armor_type` |
  | `_height`          | 目标实际高度，与宽度一起可替换`_armor_type` | 
  | `_target_2d`       | 目标2d点可替换 `_rect` |
  | `_depth`           | 自定义深度，例如：在环形高度，击打哨兵时，深度可以直接进行计算。 |

## 二、自定义函数及参数说明

---

### 初始化目标3d点

```C++
/**
 * @brief 初始化目标3d点
 * 
 * @param _armor_type 装甲板类型
 * @return std::vector<cv::Point3f> 返回目标3d点
 */
  std::vector<cv::Point3f> initialize3DPoints(int _armor_type);

  /**
 * @brief 初始化目标3d点
 * 
 * @param _width 目标实际宽度
 * @param _heigth 目标实际高度
 * @return std::vector<cv::Point3f> 返回目标3d点
 */
std::vector<cv::Point3f> initialize3DPoints(int _width, int _heigth);
```

### 初始化目标2d点

  ```C++
  /**
   * @brief 初始化目标2d点
   * 
   * @param _rect 目标旋转矩形
   * @return std::vector<cv::Point2f> 
   */
  std::vector<cv::Point2f> initialize2DPoints(cv::RotatedRect _rect);
  /**
   * @brief 初始化目标2d点
   *
   * @param _rect 目标外接矩形
   * @return std::vector<cv::Point2f>
   */
  std::vector<cv::Point2f> initialize2DPoints(cv::Rect _rect);
  ```

### 外接矩形转旋转矩形

  ```C++
  /**
   * @brief 外接矩形转旋转矩形
   * 
   * @param _rect 
   * @return cv::RotatedRect 
   */
  cv::RotatedRect rectChangeRotatedrect(cv::Rect _rect)
  ```

### 转换坐标系

  ```C++
  /**
   * @brief 转换坐标系
   *
   * @param _t 旋转向量
   * @return cv::Mat 返回转化后的旋转向量
   */
  cv::Mat cameraPtz(cv::Mat& _t);
  ```

### 绘制坐标系

  ```C++
  /**
   * @brief 绘制坐标系
   * 
   * @param _draw_img 画板
   * @param _rvec     旋转矩阵
   * @param _tvec     旋转向量
   * @param _cameraMatrix 相机内参
   * @param _distcoeffs   相机外参
   */
  void drawCoordinate(cv::Mat& _draw_img, cv::Mat& _rvec, cv::Mat& _tvec,cv::Mat& _cameraMatrix, cv::Mat& _distcoeffs);
  ```

### 计算子弹下坠

  ```C++
    /**
   * @brief 计算子弹下坠
   * 
   * @param _dist 目标深度
   * @param _tvec_y 目标高度 
   * @param _ballet_speed 子弹速度
   * @param _company 计算单位
   * @return float 返回补偿角度
   */
  float getPitch(float _dist, float _tvec_y, float _ballet_speed, const int _company = 1);
  ```

### 计算预测量

  ```C++
  /**
   * @brief 计算云台偏差角度
   * 
   * @param _pos_in_ptz 旋转向量
   * @param _bullet_speed 子弹速度
   * @param _company 计算子弹下坠单位
   * @return cv::Point3f 返回Yaw Pitch轴的偏移量和深度（mm）
   */
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz, const int _bullet_speed, const int _company);
  /**
   * @brief 计算云台偏差角度
   *
   * @param _pos_in_ptz 旋转向量
   * @param _bullet_speed 子弹速度
   * @param _company 子弹下坠单位
   * @param _depth 距目标的深度
   * @return cv::Point3f 返回Yaw Pitch轴的偏移量和深度（mm）
   */
  cv::Point3f getAngle(const cv::Mat& _pos_in_ptz, const int _bullet_speed, const int _company, const int _depth);

  ```

