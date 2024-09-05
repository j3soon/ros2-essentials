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

#ifndef INTERBOTIX_FOOTSWITCH_DRIVER__FOOTSWITCH_DRIVER_HPP_
#define INTERBOTIX_FOOTSWITCH_DRIVER__FOOTSWITCH_DRIVER_HPP_

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hidapi/hidapi.h"
#include "interbotix_footswitch_msgs/msg/footswitch_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace footswitch_driver
{

const uint16_t VEND_ID = 0x3553;
const uint16_t PROD_ID = 0xb001;

using interbotix_footswitch_msgs::msg::FootswitchState;
class FootSwitch : public rclcpp::Node
{
public:
  explicit FootSwitch(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FootSwitch();

private:
  rclcpp::TimerBase::SharedPtr tmr_update_state_;
  hid_device * handle_;
  rclcpp::Publisher<FootswitchState>::SharedPtr pub_footswitch_state_;
  int update_period_ms_{100};  // ms
  void update_state();
};

}  // namespace footswitch_driver

#endif  // INTERBOTIX_FOOTSWITCH_DRIVER__FOOTSWITCH_DRIVER_HPP_
