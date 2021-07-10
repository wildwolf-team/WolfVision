#pragma once

#include <fcntl.h>
#include <string.h>
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
  BLUE,
  RED,
};

// Description of operation mode information
enum RunMode {
  DEFAULT_MODE,
  // Self-Scanning Mode

  SUP_SHOOT,

  // Talisman Mode
  ENERGY_AGENCY,

  SENTRY_MODE,
  BASE_MODE,
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
  int set_baudrate            = 0;
  int show_serial_information = 0;
} Serial_Cfg;

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
    my_color                             = RED;
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

  void getDataForCRC(const int& data_type, const int& is_shooting,
                     const int& _yaw,      const int16_t& yaw,
                     const int& _pitch,    const int16_t& pitch,
                     const int16_t& depth);

  void getDataForSend(const int& data_type, const int& is_shooting,
                      const int& _yaw,      const int16_t& yaw,
                      const int& _pitch,    const int16_t& pitch,
                      const int16_t& depth, const uint8_t& CRC);

 public:
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
  void writeData(const Write_Data _write_data);

  void updataWriteData(const float _yaw,   const float _pitch,
                       const int   _depth, const int   _data_type = 0,
                       const int   _is_shooting = 0);

  Write_Data gainWriteData(const float _yaw,  const float _pitch,
                           const int  _depth, const int   _data_type = 0,
                           const int  _is_shooting = 0);

  void receiveData();

  bool isEmpty();

  void updateReceiveInformation();
};

}  // namespace uart
