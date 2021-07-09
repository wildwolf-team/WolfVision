#pragma once

#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
namespace abstract_pnp {
struct Solvepnp_Cfg {
  int company = 1;

  int draw_xyz = 0;

  double ptz_camera_x = 0.0;
  double ptz_camera_y = 0.0;
  double ptz_camera_z = 0.0;

  float barrel_ptz_offset_x = 0.0;
  float barrel_ptz_offset_y = 0.0;

  float offset_armor_pitch = 0.0;
  float offset_armor_yaw = 0.0;
};

/**
 * @brief 装甲板实际长度单位（mm）
 *
 */
enum ARMOR {
  // 小装甲板
  SMALL_ARMOR_HEIGHT = 60,
  SMALL_ARMOR_WIDTH = 140,
  // 大装甲板
  BIG_ARMOR_WIDTH = 245,
  BIG_ARMOR_HEIGHT = 60,
  // 灯条
  LIGHT_SIZE_W = 10,
  LIGHT_SIZE_H = 55,
  // 大神符
  BUFF_ARMOR_WIDTH = 250,
  BUFF_ARMOR_HEIGHT = 65,
};

class Abstract_Solvepnp {
 private:
  cv::Mat pnp_config_trackbar_ = cv::Mat::zeros(1, 300, CV_8UC1);
  std::vector<cv::Point3f> reference_Obj_;
  double theta = 0.0;
  double r_data[9];
  double t_data[3];
  cv::Mat r_camera_ptz;
  cv::Mat t_camera_ptz;

 public:
  Solvepnp_Cfg pnp_config_;
  Abstract_Solvepnp() {
    reference_Obj_.push_back(cv::Point3f(0.0, 0.0, 0.0));
    reference_Obj_.push_back(cv::Point3f(100, 0.0, 0.0));
    reference_Obj_.push_back(cv::Point3f(0.0, 100, 0.0));
    reference_Obj_.push_back(cv::Point3f(0.0, 0.0, 100));
    // 设相机坐标系绕X轴你是逆时针旋转θ后与云台坐标系的各个轴向平行
    static double theta = 0;
    static double r_data[] = {1,          0, 0,           0,         cos(theta),
                              sin(theta), 0, -sin(theta), cos(theta)};
    // 设相机坐标系的原点在云台坐标系中的坐标为(x0,y0,z0)
    static double t_data[] = {static_cast<double>(pnp_config_.ptz_camera_x),
                              static_cast<double>(pnp_config_.ptz_camera_y),
                              static_cast<double>(pnp_config_.ptz_camera_z)};

    r_camera_ptz = cv::Mat(3, 3, CV_64FC1, r_data);
    t_camera_ptz = cv::Mat(3, 1, CV_64FC1, t_data);
  }
  ~Abstract_Solvepnp() = default;
  /**
   * @brief 初始化3d点
   *
   * @param _armor_type
   * 默认小装甲板
   * 0 小装甲板
   * 1 大装甲板
   * 2 大神符
   * @return std::vector<cv::Point3f>
   */
  std::vector<cv::Point3f> initialize_3d_Points(int _armor_type) {
    std::vector<cv::Point3f> object_3d;
    float half_x;
    float half_y;
    // 判断赋值
    switch (_armor_type) {
      case 0:
        half_x = ARMOR::SMALL_ARMOR_WIDTH * 0.5;
        half_y = ARMOR::SMALL_ARMOR_HEIGHT * 0.5;
        break;
      case 1:
        half_x = ARMOR::BIG_ARMOR_WIDTH * 0.5;
        half_y = ARMOR::BIG_ARMOR_HEIGHT * 0.5;
        break;
      case 2:
        half_x = ARMOR::BUFF_ARMOR_WIDTH * 0.5;
        half_y = ARMOR::BUFF_ARMOR_HEIGHT * 0.5;
        break;
      default:
        half_x = ARMOR::SMALL_ARMOR_WIDTH * 0.5;
        half_y = ARMOR::SMALL_ARMOR_HEIGHT * 0.5;
        break;
    }

    // 赋值
    object_3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    object_3d.push_back(cv::Point3f(half_x, -half_y, 0));
    object_3d.push_back(cv::Point3f(half_x, half_y, 0));
    object_3d.push_back(cv::Point3f(-half_x, half_y, 0));
    return object_3d;
  }
  /**
   * @brief 初始化3d点
   *
   * @param _object_3d  3d点
   * @param _width      实际宽度
   * @param _heigth     实际高度
   * @return std::vector<cv::Point3f>
   */
  std::vector<cv::Point3f> initialize_3d_Points(int _width, int _heigth) {
    std::vector<cv::Point3f> object_3d;
    float half_x = _width * 0.5;
    float half_y = _heigth * 0.5;

    // 赋值
    object_3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    object_3d.push_back(cv::Point3f(half_x, -half_y, 0));
    object_3d.push_back(cv::Point3f(half_x, half_y, 0));
    object_3d.push_back(cv::Point3f(-half_x, half_y, 0));
    return object_3d;
  }
  /**
   * @brief 初始化2d点
   *
   * @param _rect 传入旋转矩形
   * @return std::vector<cv::Point2f>
   */
  std::vector<cv::Point2f> initialize_2d_Points(cv::RotatedRect _rect) {
    std::vector<cv::Point2f> target2d;
    static cv::Point2f vertex[4];
    static cv::Point2f lu, ld, ru, rd;
    // box的点存储到vertex中
    _rect.points(vertex);
    // 对顶点点进行排序
    std::sort(vertex, vertex + 4,
              [](const cv::Point2f &p1, const cv::Point2f &p2) {
                return p1.x < p2.x;
              });

    if (vertex[0].y < vertex[1].y) {
      lu = vertex[0];
      ld = vertex[1];
    } else {
      lu = vertex[1];
      ld = vertex[0];
    }
    if (vertex[2].y < vertex[3].y) {
      ru = vertex[2];
      rd = vertex[3];
    } else {
      ru = vertex[3];
      rd = vertex[2];
    }

    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld);
    return target2d;
  }
  /**
   * @brief 初始化2d点
   *
   * @param _rect 传入矩形
   * @return std::vector<cv::Point2f>
   */
  std::vector<cv::Point2f> initialize_2d_Points(cv::Rect _rect) {
    cv::RotatedRect box = this->rect_Change_Rotatedrect(_rect);
    return this->initialize_2d_Points(box);
  }
  /**
   * @brief 矩形转旋转矩形
   *
   * @param _rect
   * @return cv::RotatedRect
   */
  cv::RotatedRect rect_Change_Rotatedrect(cv::Rect _rect) {
    cv::RotatedRect box =
        cv::RotatedRect((_rect.tl() + _rect.br()) / 2,
                        cv::Size(_rect.width / 2, _rect.height / 2), 0);
    return box;
  }
  /**
   * @brief 计算深度信息
   * @brief: 相机与云台的X轴偏移 左负右正
   * @brief: 相机与云台的Y轴偏移 上负下正
   * @brief: 相机与云台的Z轴偏移 前正后负
   * @param _t 平移向量
   * yaw和pitch交接点相对于相机偏移量
   * @param _ptz_camera_x X轴偏移量
   * @param _ptz_camera_y Y轴偏移量
   * @param _ptz_camera_z Z轴偏移量
   *
   * @return cv::Mat
   */
  cv::Mat camera_Ptz(cv::Mat &_t) {
    cv::Mat position_in_ptz = r_camera_ptz * _t - t_camera_ptz;

    return position_in_ptz;
  }
  /**
   * @brief 绘制坐标系
   *
   * @param _draw_img 绘画板
   * @param _rvec 旋转矩阵
   * @param _tvec 平移向量
   * @param _cameraMatrix 相机内参
   * @param _distcoeffs 畸变矩阵
   */
  void draw_Coordinate(cv::Mat &_draw_img, cv::Mat &_rvec, cv::Mat &_tvec,
                       cv::Mat &_cameraMatrix, cv::Mat &_distcoeffs) {
    std::vector<cv::Point2f> reference_Img;

    cv::projectPoints(reference_Obj_, _rvec, _tvec, _cameraMatrix, _distcoeffs,
                      reference_Img);

    cv::line(_draw_img, reference_Img[0], reference_Img[1],
             cv::Scalar(0, 0, 255), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[2],
             cv::Scalar(0, 255, 0), 2);
    cv::line(_draw_img, reference_Img[0], reference_Img[3],
             cv::Scalar(255, 0, 0), 2);
    imshow("pnp_draw", _draw_img);
  }
  /**
   * @brief pitch 重力补偿 统一mm传入
   *
   * @param _dist 距离(mm)
   * @param _tvec_y 枪口与目标装甲板的垂直距离(mm)
   * @param _ballet_speed 子弹速度(mm)
   * @param _company 重力补偿算法单位
   * 1 mm
   * 10 cm
   * 100 dm
   * 1000 m
   * @return float
   */
  float get_Pitch(float _dist, float _tvec_y, float _ballet_speed,
                  const int _company = 1) {
    _dist /= _company;
    _tvec_y /= _company;
    _ballet_speed /= _company;
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    // 重力加速度单位（mm/s^2）
    const float gravity = 10000.f / _company;
    y_temp = _tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 20; i++) {
      // 计算枪口抬升角度
      a = static_cast<float>(atan2(y_temp, _dist));
      // 计算实际落点
      float t;
      t = _dist / _ballet_speed * cos(a);
      y_actual = _ballet_speed * sin(a) * t - gravity * t * t / 2;
      dy = _tvec_y - y_actual;
      y_temp = y_temp + dy;
      // 当枪口抬升角度与实际落点误差较小时退出
      if (fabsf(dy) < 0.01) {
        break;
      }
    }
    return a;
  }
  /**
   * @brief 计算yaw和pitch偏移量和depth
   *
   * @param _pos_in_ptz 世界坐标系
   * @param _bullet_speed 子弹速度
   * @param _company 重力补偿单位
   * 1 mm 10 cm 100 dm 1000 m
   * @param _barrel_ptz_offset_x 云台与枪管的X轴偏移
   * @param _barrel_ptz_offset_y 云台与枪管的Y轴偏移
   * @return cv::Point3f
   */
  cv::Point3f get_Angle(const cv::Mat &_pos_in_ptz, const int _bullet_speed,
                        const int _company, const float _barrel_ptz_offset_x,
                        const float _barrel_ptz_offset_y) {
    cv::Point3f angle;
    // 计算子弹下坠补偿时间
    const double *_xyz = (const double *)_pos_in_ptz.data;
    double down_t = 0.0;
    if (_bullet_speed > 10e-3) {
      down_t = _xyz[2] / (_bullet_speed * 1000);
    }

    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 1000;
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};  // !!!!!

    // 计算角度
    if (_barrel_ptz_offset_y != 0.f) {
      double alpha = 0.0, Beta = 0.0;
      alpha = asin(static_cast<double>(_barrel_ptz_offset_y) /
                   sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

      if (xyz[1] < 0) {
        Beta = atan(-xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha + Beta));  // camera coordinate
      } else if (xyz[1] < static_cast<double>(_barrel_ptz_offset_y)) {
        Beta = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>(-(alpha - Beta));
      } else {
        Beta = atan(xyz[1] / xyz[2]);
        angle.y = static_cast<float>((Beta - alpha));  // camera coordinate
      }
    } else {
      angle.y = static_cast<float>(atan2(xyz[1], xyz[2]));
    }
    if (_barrel_ptz_offset_x != 0.f) {
      double alpha = 0.0, Beta = 0.0;
      alpha = asin(static_cast<double>(_barrel_ptz_offset_x) /
                   sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));
      if (xyz[0] > 0) {
        Beta = atan(-xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha + Beta));  // camera coordinate
      } else if (xyz[0] < static_cast<double>(_barrel_ptz_offset_x)) {
        Beta = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(-(alpha - Beta));
      } else {
        Beta = atan(xyz[0] / xyz[2]);
        angle.x = static_cast<float>(Beta - alpha);  // camera coordinate
      }
    } else {
      angle.x = static_cast<float>(atan2(xyz[0], xyz[2]));
    }
    // depth
    angle.z = static_cast<float>(xyz[2]);
    // yaw
    angle.x = static_cast<float>(angle.x) * 180 / CV_PI;
    // pitch
    angle.y = static_cast<float>(angle.y) * 180 / CV_PI;
    angle.y -= this->get_Pitch(xyz[2], xyz[1], _bullet_speed * 1000, _company);

    return angle;
  }
};

}  // namespace abstract_pnp
