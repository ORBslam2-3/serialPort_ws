/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "serialPort/sbus.h"  // NOLINT
#include <cstddef>
#include <cstdint>
// #include "core/core.h"
#include <iostream>

bool SbusRx::Read() {
  /* Read through all available packets to get the newest */
  new_data_ = false;
  do {
    if (Parse()) {
      new_data_ = true;
    }
  } while (ser_->available());
  return new_data_;
}

bool SbusRx::Parse() {
  /* Parse messages */
  while (ser_->available()) {
    if(ser_->read(&cur_byte_,1)==1)
    {
      // std::cout<<"当前 得到的 header ： "<<std::hex<<cur_byte_<<std::endl;
    if (state_ == 0) {
      if ((cur_byte_ == HEADER_) && (prev_byte_ == FOOTER_)) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
      }
    } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_) {
        buf_[state_++] = cur_byte_;
        // std::cout<<"每帧数据state_ ： "<<state_<<std::endl;
    } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_) {
      state_ = 0;
      prev_byte_ = cur_byte_;
      // std::cout<<"每帧数据开头 ： "<<buf_[0]<<std::endl;
      if (cur_byte_ == FOOTER_) {
        /* Grab the channel data */
        //ch[0]为左右值，ch[2]为前后值
        data_.ch[0]  = static_cast<int16_t>(buf_[1] |
                                            ((buf_[2] << 8) & 0x07FF));
        data_.ch[1]  = static_cast<int16_t>((buf_[2] >> 3) |
                                            ((buf_[3] << 5) & 0x07FF));
        data_.ch[2]  = static_cast<int16_t>((buf_[3] >> 6) |
                                            (buf_[4] << 2) |
                                            ((buf_[5] << 10) & 0x07FF));
        data_.ch[3]  = static_cast<int16_t>((buf_[5] >> 1) |
                                            ((buf_[6] << 7) & 0x07FF));
        data_.ch[4]  = static_cast<int16_t>((buf_[6] >> 4) |
                                            ((buf_[7] << 4) & 0x07FF));
        data_.ch[5]  = static_cast<int16_t>((buf_[7] >> 7) |
                                            (buf_[8] << 1) |
                                            ((buf_[9] << 9) & 0x07FF));
        data_.ch[6]  = static_cast<int16_t>((buf_[9] >> 2) |
                                            ((buf_[10] << 6) & 0x07FF));
        data_.ch[7]  = static_cast<int16_t>((buf_[10] >> 5) |
                                            ((buf_[11] << 3) & 0x07FF));
        data_.ch[8]  = static_cast<int16_t>(buf_[12] |
                                            ((buf_[13] << 8) & 0x07FF));
        data_.ch[9]  = static_cast<int16_t>((buf_[13] >> 3) |
                                            ((buf_[14] << 5) & 0x07FF));
        data_.ch[10] = static_cast<int16_t>((buf_[14] >> 6) |
                                            (buf_[15] << 2) |
                                            ((buf_[16] << 10) & 0x07FF));
        data_.ch[11] = static_cast<int16_t>((buf_[16] >> 1) |
                                            ((buf_[17] << 7) & 0x07FF));
        data_.ch[12] = static_cast<int16_t>((buf_[17] >> 4) |
                                            ((buf_[18] << 4) & 0x07FF));
        data_.ch[13] = static_cast<int16_t>((buf_[18] >> 7) |
                                            (buf_[19] << 1) |
                                            ((buf_[20] << 9) & 0x07FF));
        data_.ch[14] = static_cast<int16_t>((buf_[20] >> 2) |
                                            ((buf_[21] << 6) & 0x07FF));
        data_.ch[15] = static_cast<int16_t>((buf_[21] >> 5) |
                                            ((buf_[22] << 3) & 0x07FF));
        /* CH 17 */
        data_.ch17 = buf_[23] & CH17_MASK_;
        /* CH 18 */
        data_.ch18 = buf_[23] & CH18_MASK_;
        /* Grab the lost frame */
        data_.lost_frame = buf_[23] & LOST_FRAME_MASK_;
        /* Grab the failsafe */
        data_.failsafe = buf_[23] & FAILSAFE_MASK_;
        return true;
      } else {
        return false;
      }
    } else {
      state_ = 0;
    }
    prev_byte_ = cur_byte_;
    }
  }
  return false;
}


