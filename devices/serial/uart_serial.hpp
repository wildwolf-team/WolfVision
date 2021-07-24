#pragma once

#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/opencv.hpp>

namespace uart {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "uart_serial");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "uart_serial");

enum BufferLength {
  // The recieve length of the array obtained after decoding
  REC_INFO_LENGTH   = 16,

  // The send length of the array for CRC auth code code calculating
  CRC_BUFF_LENGTH   = 11,

  // The send length of the array after append CRC auth code
  WRITE_BUFF_LENGTH = 13,
};

// The color of our team
enum Color {
  ALL,
  RED,
  BLUE,
};

// Description of operation mode information
enum RunMode {
  DEFAULT_MODE,
  // Self-Scanning Mode
  SUP_SHOOT,
  // Talisman Mode
  ENERGY_AGENCY,
  // Hitting the sentry Mode
  SENTRY_STRIKE_MODE,
  // Little top mode
  TOP_MODE,
  // Record video Mode
  RECORD_MODE,
  // Plane Mode
  PLANE_MODE,
  // Sentrys autonomous mode
  SENTINEL_AUTONOMOUS_MODE,
  // Radar Mode
  RADAR_MODE,
};
// Describe the current robot ID information
enum RobotID {
  HERO = 1,
  UAV  = 6,
  ENGINEERING,
  INFANTRY,
  SENTRY,
};

struct Serial_Config {
  std::string preferred_device        = "/dev/ttyUSB0";
  int         set_baudrate            = 0;
  int         show_serial_information = 0;
};

// Serial port information receiving structure
struct Receive_Data {
  int   my_color;
  int   now_run_mode;
  int   my_robot_id;
  int   bullet_velocity;
  float acceleration;

  // Description of the yaw axis angle of the gyroscope (signed)
  union Receive_Yaw_Angle_Information {
    float   yaw_angle;
    uint8_t arr_yaw_angle[4] = {0};
  } Receive_Yaw_Angle_Info;

  // Description of the pitch axis angle of the gyroscope (signed)
  union Receive_Pitch_Angle_Information {
    float   pitch_angle;
    uint8_t arr_pitch_angle[4] = {0};
  } Receive_Pitch_Angle_Info;

  Receive_Data() {
    my_color                             = BLUE;
    now_run_mode                         = SUP_SHOOT;
    my_robot_id                          = INFANTRY;
    acceleration                         = 0.f;
    bullet_velocity                      = 30;
    Receive_Yaw_Angle_Info.yaw_angle     = 0.f;
    Receive_Pitch_Angle_Info.pitch_angle = 0.f;
  }
};

// Serial port message sending structure
struct Write_Data {
  int   symbol_yaw;
  int   symbol_pitch;
  int   depth;

  // Whether the robot shold shoot, 1 for shoot for 1 ball, otherwises 0
  int   is_shooting;

  // Whether the target is found, 1 for found, otherwise 0
  int   data_type;

  float yaw_angle;
  float pitch_angle;

  Write_Data() {
    symbol_yaw   = 0;
    symbol_pitch = 0;
    depth        = 0;
    is_shooting  = 0;
    data_type    = 0;
    yaw_angle    = 0.f;
    pitch_angle  = 0.f;
  }
};

class SerialPort {
 public:
  SerialPort() = default;
  explicit SerialPort(std::string _serial_config);

  ~SerialPort();

  inline Receive_Data returnReceive() { return receive_data_; }

  inline float returnReceiveAcceleration()   { return receive_data_.acceleration; }
  inline int   returnReceiveBulletVelocity() { return receive_data_.bullet_velocity; }
  inline int   returnReceiveRobotId()        { return receive_data_.my_robot_id; }
  inline int   returnReceiceColor()          { return receive_data_.my_color; }
  inline int   returnReceiveMode()           { return receive_data_.now_run_mode; }
  inline float returnReceivePitch()          { return receive_data_.Receive_Pitch_Angle_Info.pitch_angle; }
  inline float receiveYaw()                  { return receive_data_.Receive_Yaw_Angle_Info.yaw_angle; }

  inline unsigned char returnHighBit(const int& Byte) {
    exchangebyte_ = (Byte >> 8) & 0xff;

    return exchangebyte_;
  }

  inline unsigned char returnLowBit(const int& Byte) {
    exchangebyte_ = Byte & 0xff;

    return exchangebyte_;
  }


  inline int16_t mergeIntoBytes(const unsigned char& highbit,
                                const unsigned char& lowbit) {
    exchangebit_ = (highbit << 8) | lowbit;

    return exchangebit_;
  }

  void writeData(const int&     _yaw,   const int16_t& yaw,
                 const int&     _pitch, const int16_t& pitch,
                 const int16_t& depth,  const int&     data_type = 0,
                 const int&     s_shooting = 0);
  void writeData();
  void writeData(const Write_Data& _write_data);

  void updataWriteData(const float _yaw,   const float _pitch,
                       const int   _depth, const int   _data_type = 0,
                       const int   _is_shooting = 0);

  Write_Data gainWriteData(const float _yaw,  const float _pitch,
                           const int  _depth, const int   _data_type = 0,
                           const int  _is_shooting = 0);

  void receiveData();

  bool isEmpty();

  void updateReceiveInformation();

 private:
  Serial_Config serial_config_;
  Receive_Data  receive_data_;
  Receive_Data  last_receive_data_;
  Write_Data    write_data_;

  int           fd;
  int           transform_arr_[4];
  unsigned char write_buff_[WRITE_BUFF_LENGTH];
  unsigned char crc_buff_[CRC_BUFF_LENGTH];
  unsigned char receive_buff_[REC_INFO_LENGTH];
  unsigned char receive_buff_temp_[REC_INFO_LENGTH * 2];
  unsigned char exchangebyte_;

  int16_t yaw_reduction_;
  int16_t pitch_reduction_;
  int16_t depth_reduction_;

  int16_t angle_reduction_;
  int16_t acceleration_reduction_;

  int16_t exchangebit_;

  ssize_t read_message_;
  ssize_t write_message_;

  inline uint8_t checksumCRC(unsigned char* buf, uint16_t len);

  void getDataForCRC(const int&     data_type, const int&     is_shooting,
                     const int&     _yaw,      const int16_t& yaw,
                     const int&     _pitch,    const int16_t& pitch,
                     const int16_t& depth);

  void getDataForSend(const int&     data_type, const int&     is_shooting,
                      const int&     _yaw,      const int16_t& yaw,
                      const int&     _pitch,    const int16_t& pitch,
                      const int16_t& depth, const uint8_t&     CRC);
};

static constexpr unsigned char CRC8_Table[] = {
  0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
  65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
  130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
  222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
  29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
  102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
  165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
  249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
  58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
  144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
  83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
  40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
  235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
  183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
  116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
  53
};

}  // namespace uart
