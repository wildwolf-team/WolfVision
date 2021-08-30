# Basic_Roi 使用手册

## 一、 使用说明


### 头文件说明

- 包含头文件 `basic_roi.hpp`

| 文件名                  | 文件说明                                   |
| ---------------------- | ------------------------------------------- |
| `abstract_roi.hpp`     | ROI 抽象类           |
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

## 二、 自定义函数说明


### 保证 ROI 不超过图片范围

```cpp
  cv::Rect makeRectSafeFixed(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);
  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::RotatedRect& _r_rect);
  cv::Rect makeRectSafeTailor(const cv::Mat& _input_img,
                              const cv::Rect& _r_rect);
  cv::Rect makeRectSafeThird(const cv::Mat& _input_img,
                             const cv::RotatedRect& _r_rect);
```

  设计思路:  
  把 ROI 裁剪数值限定到 _input_img 图片边框数值范围内，防止 ROI 裁剪超出 _input_img 的边界而产生的程序错误。  

### 返回 ROI 图像

  ```cpp
  cv::Mat returnROIResultMat(const cv::Mat& _input_img);
  ```

  设计思路:    
  为了防止画面中出现多个装甲板时，最优装甲板位置判断反复横跳。当自瞄在当前帧检测到某一个装甲板为最优目标装甲板，保存该装甲板在图片上的外接矩阵信息，下一帧根据保存的外接矩阵信息截取特定 ROI 图片传入自瞄。若传入的 ROI 图片检测不到装甲板，则逐渐扩大 ROI 图片的大小。  
  
 
### 使用方法请参考
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
