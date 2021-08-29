# Basic_Roi 使用手册

## 一、 使用说明

---

### 头文件说明

- 包含头文件`basic_roi.hpp`

| 文件名                  | 文件说明                                   |
| ---------------------- | ------------------------------------------- |
| `abstract_roi.hpp`     | 角度结算抽象类           |
### 实例化对象

实例化对象时调用构造函数：`RoI();`
  
### 函数说明

在模式选择后调用接口函数即可：

```cpp
cv::Rect makeRectSafeFixed(const cv::Mat& _input_img, const cv::RotatedRect& _r_rect);
cv::Rect makeRectSafeTailor(const cv::Mat& _input_img, const cv::RotatedRect& _r_rect);
cv::Rect makeRectSafeTailor(const cv::Mat& _input_img, const cv::Rect& _r_rect);
cv::Rect makeRectSafeThird(const cv::Mat& _input_img, const cv::RotatedRect& _r_rect);
```
参数解释：
|      参数名         |           参数解释             |
| ------------------ | -----------------------------|
| `_src_img`         | 相机输入的图像，如：`src_img`|
| `_r_rect`          | 需要 ROI 的范围|

## 二、 自定义函数及参数说明

---

### 保证 ROI 不超过图片范围

```cpp
  /**
   * @brief 限制 ROI 范围
   * 
   * @param _input_img 图像
   * @param _r_rect    ROI 区域
   * @return cv::Rect  返回安全的 Rect 参数
   */
  cv::Rect makeRectSafeFixed(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);
  /**
   * @brief 限制 ROI 范围
   *
   * @param _input_img 图像
   * @param _r_rect    ROI 区域
   * @return cv::Rect 返回安全的 Rect 参数
   */
  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::RotatedRect& _r_rect);
  /**
   * @brief 限制 ROI 范围
   *
   * @param _input_img 图像
   * @param _r_rect    ROI 区域
   * @return cv::Rect  返回安全的 Rect 参数
   */
  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::Rect& _r_rect);
  /**
   * @brief 限制 ROI 范围
   *
   * @param _input_img 图像
   * @param _r_rect    ROI 区域
   * @return cv::Rect  返回安全的 Rect 参数
   */
  cv::Rect makeRectSafeThird(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);
```

### 返回ROI图像

  ```cpp
  /**
   * @brief 返回 ROI 图像（逐级扩大 ROI 范围） 
   * 
   * @param _input_img 图像
   * @return cv::Mat   ROI 图像
   */
  cv::Mat returnROIResultMat(const cv::Mat& _input_img);
  ```
#### 使用方法请参考
  ```cpp
  roi_img_ = roi_.returnROIResultMat(src_img_);
  if (basic_armor_.runBasicArmor(roi_img_, serial_.returnReceive())) {
    basic_armor_.fixFinalArmorCenter(0, roi_.returnRectTl());
      roi_.setLastRoiRect(basic_armor_.returnFinalArmorRotatedRect(0),
                          basic_armor_.returnFinalArmorDistinguish(0));
      pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                    basic_armor_.returnFinalArmorDistinguish(0),
                    basic_armor_.returnFinalArmorRotatedRect(0));
    serial_.updataWriteData(pnp_.returnYawAngle(),
                            pnp_.returnPitchAngle(),
                            pnp_.returnDepth(),
                            basic_armor_.returnArmorNum(), 0);
  } else {
    serial_.updataWriteData(-pnp_.returnYawAngle(),
                            pnp_.returnPitchAngle(),
                            pnp_.returnDepth(),
                            basic_armor_.returnLostCnt() > 0 ? 1 : 0,
                            0);
  }
  roi_.setLastRoiSuccess(basic_armor_.returnArmorNum());
  ```