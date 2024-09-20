// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERBOTIX_SLATE_DRIVER__SERIAL_DRIVER_HPP_
#define INTERBOTIX_SLATE_DRIVER__SERIAL_DRIVER_HPP_

#include <termio.h>

#include <mutex>
#include <string>

typedef enum
{
  SYS_INIT = 0x00,
  SYS_NORMAL,
  SYS_REMOTE,
  SYS_ESTOP,
  SYS_CALIB,
  SYS_TEST,
  SYS_CHARGING,

  SYS_ERR = 0x10,
  SYS_ERR_ID,
  SYS_ERR_COM,
  SYS_ERR_ENC,
  SYS_ERR_COLLISION,
  SYS_ERR_LOW_VOLTAGE,
  SYS_ERR_OVER_VOLTAGE,
  SYS_ERR_OVER_CURRENT,
  SYS_ERR_OVER_TEMP,

  SYS_STATE_LEN,
} SystemState;

class SerialDriver
{
 public:
  ~SerialDriver();
  bool init(std::string& port, uint8_t id, int baud = B115200);

  bool getVersion(char* version);
  bool setText(const char* text);

  int readHoldingRegs(uint16_t addr, uint16_t cnt, uint16_t* data);
  int writeHoldingRegs(uint16_t addr, uint16_t cnt, uint16_t* data);
  int readWriteHoldingRegs(uint16_t raddr, uint16_t rcnt, uint16_t* rdata,
                           uint16_t waddr, uint16_t wcnt, uint16_t* wdata);

  int send(const void* data, int len, int timeout = 0);
  int recv(uint8_t* data, int maxlen, int timeout = 0);

 private:
  void* m = nullptr;
  int fd_ = -1;
  fd_set read_set_;
  std::mutex lock;
};

#endif  // INTERBOTIX_SLATE_DRIVER__SERIAL_DRIVER_HPP_
