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

#ifndef INTERBOTIX_XS_ROS_CONTROL__XS_HARDWARE_INTERFACE_HPP_
#define INTERBOTIX_XS_ROS_CONTROL__XS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using RobotInfo = interbotix_xs_msgs::srv::RobotInfo;
using hardware_interface::HW_IF_POSITION;

namespace interbotix_xs_ros_control
{

class XSHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XSHardwareInterface)

  /// @brief Initializes the XSHardwareInterface.
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /// @brief Exports all state interfaces for the XSHardwareInterface.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// @brief Exports all command interfaces for the XSHardwareInterface.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// @brief Start this
  CallbackReturn start();

  /// @brief Stop this
  CallbackReturn stop();

  /// @brief Read data, updates joint_position vector from the joint_states
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  /// @brief Write data, publishes to the /commands/ topics
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  /// @brief Returns the name of this hardware interface
  /// @return The name of this hardware interface
  std::string get_name() const final
  {
    return info_.name;
  }

  /// @brief ROS Subscriber callback that stores joint_states
  /// @param msg The JointState message from the joint_states topic
  void joint_state_cb(const sensor_msgs::msg::JointState & msg);

protected:
  // Joint positions populated by the joint_states topic
  std::vector<double> joint_positions;

  // Joint velocities populated by the joint_states topic
  std::vector<double> joint_velocities;

  // Joint efforts populated by the joint_states topic
  std::vector<double> joint_efforts;

  // Position commands for arm group, published to the /commands/joint_group topic
  std::vector<double> joint_position_commands;

  // Position commands for the gripper joint, published to the /commands/joint_single
  float gripper_cmd_prev;

  // Previous update's joint commands
  std::vector<float> joint_commands_prev;

  /// @brief Function allowing the executor to spin in another thread
  void executor_cb();

  // Thread the executor_cb function runs in
  std::thread update_thread;

  // ROS Node for this hardware interface's pubs, subs, and services
  std::shared_ptr<rclcpp::Node> nh;

  // Executor to update this hardware interface's node
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  // Publishes arm group commands to the /commands/joint_group topic
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_group;

  // Publishes gripper commands to the /commands/joint_single topic
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr pub_gripper;

  // Subscribes to the joint_states topic, keeps the joint_* vectors updated
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states;

  // Client for the /get_robot_info service
  rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr srv_robot_info;

  // Parameters from the ros2_control tag
  HardwareInfo info_;

  // Name of the arm group, populated by the info_ HardwareInfo
  std::string group_name;

  // Name of the gripper joint, populated by the info_ HardwareInfo
  std::string gripper_name;

  // Helps map joints to their index
  std::vector<int16_t> joint_state_indices;

  // Mutex that locks the joint_state info
  std::mutex joint_state_mtx_;

  // The number of joints in the robot
  size_t num_joints;

  // joint_states message from the joint_states topic
  sensor_msgs::msg::JointState joint_states;
};

}  // namespace interbotix_xs_ros_control

#endif  // INTERBOTIX_XS_ROS_CONTROL__XS_HARDWARE_INTERFACE_HPP_
