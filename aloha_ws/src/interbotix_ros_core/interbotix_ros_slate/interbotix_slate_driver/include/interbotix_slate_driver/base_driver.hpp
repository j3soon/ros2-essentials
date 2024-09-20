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

#ifndef INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_HPP_
#define INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_HPP_

#include <string>

#include "interbotix_slate_driver/serial_driver.hpp"


namespace base_driver
{

typedef struct
{
  float cmd_vel_x;
  float cmd_vel_y;
  float cmd_vel_z;
  uint32_t light_state;

  uint32_t system_state;
  uint32_t charge;
  float voltage;
  float current;
  float vel_x;
  float vel_y;
  float vel_z;
  float odom_x;
  float odom_y;
  float odom_z;
  uint32_t cmd;
  uint32_t io;
  uint32_t err;
} ChassisData;

bool chassisInit(std::string &dev);
bool getVersion(char *data);
bool setText(const char *text);
bool updateChassisInfo(ChassisData *data);
bool setSysCmd(uint32_t cmd);
bool setIo(uint32_t io);

}  // namespace base_driver

#endif  // INTERBOTIX_SLATE_DRIVER__BASE_DRIVER_HPP_
