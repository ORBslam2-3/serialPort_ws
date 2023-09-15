/*
* 解析s.bus协议
1. 串口配置：100k波特率，8位数据位（在stm32中要选择9位），偶校验 （EVEN),2位停止位，无控流，25个字节。

2. 协议格式：（每字节8位）
   [startbyte] [data1][data2]…[data22][flags][endbyte]；
   startbyte=0x0f；
   endbyte=0x00；

3. flags标志位是用来检测控制器与px4是否断开的标志位：
   flags=1：控制器与接收器保持连接；
   flags=0：控制器与接收器断开（失控），px4会控制电机停转。
4. 

*/

#ifndef SRC_SBUS_H_
#define SRC_SBUS_H_

#include <cstddef>
#include <cstdint>
// #include "core/core.h"
#include <serial/serial.h>

struct SbusData {
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  static constexpr int8_t NUM_CH = 16;
  int16_t ch[NUM_CH];
};

class SbusRx {
 public:
  explicit SbusRx(serial::Serial  *ser) : ser_(ser) {}
  bool Read();
  inline SbusData data() const {return data_;}
  bool Parse();
 private:
  /* Communication */
  serial::Serial *ser_;
  bool inv_ = true;
  bool fast_ = false;
  int32_t baud_ = 100000;
  /* Message len */
  static constexpr int8_t PAYLOAD_LEN_ = 23;
  static constexpr int8_t HEADER_LEN_ = 1;
  static constexpr int8_t FOOTER_LEN_ = 1;
  /* SBUS message defs */
  static constexpr int8_t NUM_SBUS_CH_ = 16;
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t FOOTER2_ = 0x00;
  static constexpr uint8_t CH17_MASK_ = 0x01;
  static constexpr uint8_t CH18_MASK_ = 0x02;
  static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
  static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
  /* Parsing state tracking */
  int state_ = 0;
  int prev_byte_ = FOOTER_;
  uint8_t cur_byte_;
  /* Buffer for storing messages */
  int buf_[25];
  /* Data */
  bool new_data_;
  SbusData data_;
};

#endif  // SRC_SBUS_H_
