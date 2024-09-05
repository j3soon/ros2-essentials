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

#include <interbotix_xs_ros_control/xs_hardware_interface.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace interbotix_xs_ros_control
{

CallbackReturn XSHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  nh = std::make_shared<rclcpp::Node>("xs_hardware_interface");
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(nh);

  // get hardware parameters
  group_name = info_.hardware_parameters["group_name"];
  gripper_name = info_.hardware_parameters["gripper_name"];
  std::string js_topic = info_.hardware_parameters["joint_states_topic"];

  // create pubs, subs, and services
  pub_group = nh->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
    "commands/joint_group",
    1);
  pub_gripper = nh->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>(
    "commands/joint_single",
    1);
  sub_joint_states = nh->create_subscription<sensor_msgs::msg::JointState>(
    js_topic,
    1,
    std::bind(&XSHardwareInterface::joint_state_cb, this, std::placeholders::_1));
  srv_robot_info = nh->create_client<interbotix_xs_msgs::srv::RobotInfo>("get_robot_info");

  using namespace std::chrono_literals;

  // wait for a few seconds for the xs_sdk services to load
  if (!srv_robot_info->wait_for_service(5s)) {
    RCLCPP_FATAL(
      nh->get_logger(),
      "Could not get robot_info service within timeout.");
    throw std::runtime_error("Could not get robot_info service within timeout.");
  }

  // set up RobotInfo service requests, call, and await responses
  auto group_info_srv = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  auto gripper_info_srv = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  group_info_srv->cmd_type = "group";
  group_info_srv->name = group_name;
  gripper_info_srv->cmd_type = "single";
  gripper_info_srv->name = gripper_name;
  auto group_future = srv_robot_info->async_send_request(group_info_srv);
  auto gripper_future = srv_robot_info->async_send_request(gripper_info_srv);
  executor->spin_until_future_complete(group_future);
  executor->spin_until_future_complete(gripper_future);
  auto group_res = group_future.get();
  num_joints = group_res->num_joints;
  joint_state_indices = group_res->joint_state_indices;
  auto grip_res = gripper_future.get();
  joint_state_indices.push_back(grip_res->joint_state_indices.at(0));

  // Get robot joint names from service response, configure vectors
  std::vector<std::string> joint_names = group_res->joint_names;
  joint_names.push_back(grip_res->joint_names.at(0));
  joint_positions.resize(num_joints);
  joint_velocities.resize(num_joints);
  joint_efforts.resize(num_joints);
  joint_position_commands.resize(num_joints);
  joint_commands_prev.resize(num_joints);

  // create thread that spins the executor for the pubs, subs, and services
  update_thread = std::thread(&XSHardwareInterface::executor_cb, this);

  while (joint_states.position.size() == 0 && rclcpp::ok()) {
    RCLCPP_INFO_ONCE(nh->get_logger(), "Waiting for joint states...");
  }

  RCLCPP_INFO(nh->get_logger(), "Joint states received.");

  // Initialize the joint_position_commands vector to the current joint states
  for (size_t i{0}; i < num_joints; i++) {
    joint_position_commands.at(i) = joint_states.position.at(joint_state_indices.at(i));
    joint_commands_prev.at(i) = joint_position_commands.at(i);
  }
  joint_commands_prev.resize(num_joints);
  gripper_cmd_prev = joint_states.position.at(joint_state_indices.back()) * 2;

  // Command robot to its sleep position to push it within its limits
  //  Until joint_limits_interface is merged into ros2_control repo
  //  https://github.com/ros-controls/ros2_control/pull/462
  interbotix_xs_msgs::msg::JointGroupCommand group_cmd;
  group_cmd.name = group_name;
  group_cmd.cmd = group_res->joint_sleep_positions;
  pub_group->publish(group_cmd);

  joint_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        nh->get_logger(),
        "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), static_cast<int>(joint.command_interfaces.size()));
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        nh->get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

void XSHardwareInterface::executor_cb()
{
  executor->spin();
}

CallbackReturn XSHardwareInterface::start()
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn XSHardwareInterface::stop()
{
  update_thread.join();  // make sure to join update thread before stopping
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> XSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &joint_positions[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &joint_velocities[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XSHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_commands[i]));
  }
  return command_interfaces;
}

return_type XSHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lck(joint_state_mtx_);
  for (size_t i = 0; i < num_joints; i++) {
    joint_positions.at(i) = joint_states.position.at(joint_state_indices.at(i));
  }
  return return_type::OK;
}

return_type XSHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  interbotix_xs_msgs::msg::JointGroupCommand group_msg;
  interbotix_xs_msgs::msg::JointSingleCommand gripper_msg;
  group_msg.name = group_name;
  gripper_msg.name = gripper_name;
  gripper_msg.cmd = joint_position_commands.back() * 2;

  for (size_t i{0}; i < num_joints; i++) {
    group_msg.cmd.push_back(joint_position_commands.at(i));
  }

  // Only publish commands if different than the previous update's commands
  if (joint_commands_prev != group_msg.cmd) {
    pub_group->publish(group_msg);
    joint_commands_prev = group_msg.cmd;
  }
  if (gripper_cmd_prev != gripper_msg.cmd) {
    pub_gripper->publish(gripper_msg);
    gripper_cmd_prev = gripper_msg.cmd;
  }
  return return_type::OK;
}

void XSHardwareInterface::joint_state_cb(const sensor_msgs::msg::JointState & msg)
{
  std::lock_guard<std::mutex> lck(joint_state_mtx_);
  joint_states = msg;
}

}  // namespace interbotix_xs_ros_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  interbotix_xs_ros_control::XSHardwareInterface,
  hardware_interface::SystemInterface)
