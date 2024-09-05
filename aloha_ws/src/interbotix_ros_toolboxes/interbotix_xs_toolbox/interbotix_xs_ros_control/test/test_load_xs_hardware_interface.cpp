// Copyright 2022 Trossen Robotics
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
//    * Neither the name of the the copyright holder nor the names of its
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

// inspired by https://github.com/ros-controls/ros2_control/blob/fe462926416d527d1da163bc3eabd02ee1de9be9/hardware_interface/test/fake_components/test_generic_system.cpp#L441-L446

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestXSHardwareSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_xs_ =
      R"(
  <ros2_control name="XSHardwareInterface" type="system">
    <hardware>
      <plugin>interbotix_xs_ros_control/XSHardwareInterface</plugin>
      <param name="loop_hz">10</param>
      <param name="group_name">arm</param>
      <param name="gripper_name">gripper</param>
      <param name="joint_states_topic">joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">0.7854</param>
    </joint>
  </ros2_control>
)";
  }
  std::string hardware_system_xs_;
};

TEST_F(TestXSHardwareSystem, load_hardware_system_xs_)
{
  // throws due to wait_for_service timeout
  rclcpp::init(0, nullptr);
  auto urdf =
    ros2_control_test_assets::urdf_head + hardware_system_xs_ + ros2_control_test_assets::urdf_tail;
  EXPECT_THROW(hardware_interface::ResourceManager rm(urdf), std::runtime_error);
}
