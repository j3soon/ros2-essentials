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

#include "rclcpp/rclcpp.hpp"

#include "interbotix_slate_driver/base_driver.hpp"
#include "interbotix_slate_driver/serial_driver.hpp"

namespace base_driver
{

#define MAX_TIMEOUT_CNT 50

uint32_t err_cnt = 0;
SerialDriver driver;

bool chassisInit(std::string &dev) { return driver.init(dev, 1, B115200); }

bool check(int ret)
{
  err_cnt += (ret == 0 ? 0 : 1);
  if (err_cnt > MAX_TIMEOUT_CNT)
  {
    std::string dev;
    err_cnt = 0;
  }
  return !ret;
}

bool getVersion(char *data) { return driver.getVersion(data); }
bool setText(const char *text) { return driver.setText(text); }

bool updateChassisInfo(ChassisData *data)
{
  return check(driver.readWriteHoldingRegs(0x00, 26, (uint16_t *)&(data->system_state),
                                           0x40, 8, (uint16_t *)data));
}

bool setSysCmd(uint32_t cmd)
{
  return check(driver.writeHoldingRegs(0x14, 2, (uint16_t *)&cmd));
}

bool setIo(uint32_t io)
{
  return check(driver.writeHoldingRegs(0x16, 2, (uint16_t *)&io));
}

}  // namespace base_driver
