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

#ifndef INTERBOTIX_XS_SDK__XS_SDK_OBJ_HPP_
#define INTERBOTIX_XS_SDK__XS_SDK_OBJ_HPP_

#include <string>
#include <regex>
#include <bitset>
#include <vector>
#include <algorithm>
#include <memory>
#include <unordered_map>

#include "urdf/model.h"
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "interbotix_xs_driver/xs_common.hpp"
#include "interbotix_xs_driver/xs_driver.hpp"
#include "interbotix_xs_driver/xs_logging.hpp"

#include "interbotix_xs_msgs/srv/reboot.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/motor_gains.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/register_values.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "interbotix_xs_msgs/msg/joint_trajectory_command.hpp"

namespace interbotix_xs
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("interbotix_xs_sdk.xs_sdk");

// simplify message and service usage
using MotorGains = interbotix_xs_msgs::srv::MotorGains;
using OperatingModes = interbotix_xs_msgs::srv::OperatingModes;
using Reboot = interbotix_xs_msgs::srv::Reboot;
using RegisterValues = interbotix_xs_msgs::srv::RegisterValues;
using RobotInfo = interbotix_xs_msgs::srv::RobotInfo;
using TorqueEnable = interbotix_xs_msgs::srv::TorqueEnable;
using JointGroupCommand = interbotix_xs_msgs::msg::JointGroupCommand;
using JointSingleCommand = interbotix_xs_msgs::msg::JointSingleCommand;
using JointTrajectoryCommand = interbotix_xs_msgs::msg::JointTrajectoryCommand;

/// @brief The Interbotix X-Series ROS Node
class InterbotixRobotXS : public rclcpp::Node
{
public:
  /// @brief Constructor for the InterbotixRobotXS
  /// @param succes <out> Whether or not the Node initialized properly
  /// @param options ROS NodeOptions
  explicit InterbotixRobotXS(
    bool & success,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor for the InterbotixRobotXS
  ~InterbotixRobotXS();

private:
  // Frequency at which the ROS Timer publishing joint states should run
  int timer_hz;

  // Boolean that determines if joint states should be published;
  bool pub_states;

  // Boolean that changes value when a JointTrajectoryCommand begins and ends execution
  bool execute_joint_traj;

  // InterbotixDriverXS object used to talk to the lower-level XS Interfaces
  std::unique_ptr<InterbotixDriverXS> xs_driver;

  // ROS Publisher responsible for publishing joint states
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states;

  // ROS Subscriber responsible for subscribing to JointGroupCommand messages
  rclcpp::Subscription<JointGroupCommand>::SharedPtr sub_command_group;

  // ROS Subscriber responsible for subscribing to JointSingleCommand messages
  rclcpp::Subscription<JointSingleCommand>::SharedPtr sub_command_single;

  // ROS Subscriber responsible for subscribing to JointTrajectoryCommand messages
  rclcpp::Subscription<JointTrajectoryCommand>::SharedPtr sub_command_traj;

  // ROS Service Server used to set motor PID gains
  rclcpp::Service<MotorGains>::SharedPtr srv_motor_gains;

  // ROS Service Server used to set motor operating modes
  rclcpp::Service<OperatingModes>::SharedPtr srv_operating_modes;

  // ROS Service Server used to set any motor register
  rclcpp::Service<RegisterValues>::SharedPtr srv_set_registers;

  // ROS Service Server used to get any motor register
  rclcpp::Service<RegisterValues>::SharedPtr srv_get_registers;

  // ROS Service Server used to get general information about the robot (like limits)
  rclcpp::Service<RobotInfo>::SharedPtr srv_get_robot_info;

  // ROS Service Server used to torque on/off any motor
  rclcpp::Service<TorqueEnable>::SharedPtr srv_torque_enable;

  // ROS Service Server used to reboot any motor
  rclcpp::Service<Reboot>::SharedPtr srv_reboot_motors;

  // ROS Timer used to continuously publish joint states
  rclcpp::TimerBase::SharedPtr tmr_joint_states;

  // ROS One-Shot Timer used when commanding motor trajectories
  rclcpp::TimerBase::SharedPtr tmr_joint_traj;

  // Holds the most recent JointState message
  sensor_msgs::msg::JointState joint_states;

  // Holds the most recent JointTrajectoryCommand message
  JointTrajectoryCommand::SharedPtr joint_traj_cmd;

  // Indicating the trajectory start time
  rclcpp::Time traj_start_time;

  // Trajectory counter
  size_t cntr;

  // Desired JointState topic name
  std::string js_topic;

  // Absolute path to the motor configs file
  std::string filepath_motor_configs;

  // Absolute path to the mode configs file
  std::string filepath_mode_configs;

  // Whether or not to write to the EEPROM values on startup
  bool write_eeprom_on_startup;

  // The xs_driver logging level as a string (to be set from ROS parameters)
  std::string xs_driver_logging_level;

  /// @brief Loads the X-Series robot driver
  /// @returns True if the driver was loaded successfully, False otherwise
  bool robot_init_driver();

  /// @brief Declare all parameters needed by the node
  void robot_init_parameters();

  /// @brief Initialize ROS Publishers
  void robot_init_publishers();

  /// @brief Initialize ROS Subscribers
  void robot_init_subscribers();

  /// @brief Initialize ROS Services
  void robot_init_services();

  /// @brief Initialize ROS Timers
  void robot_init_timers();

  /// @brief Waits until first JointState message is received
  void robot_wait_for_joint_states();

  /// @brief ROS Subscriber callback function to command a group of joints
  /// @param msg JointGroupCommand message dictating the joint group to command along with the
  ///   actual commands
  /// @details refer to the message definition for details
  void robot_sub_command_group(const JointGroupCommand::SharedPtr msg);

  /// @brief ROS Subscriber callback function to command a single joint
  /// @param msg JointSingleCommand message dictating the joint to command along with the actual
  ///   command
  /// @details refer to the message definition for details
  void robot_sub_command_single(const JointSingleCommand::SharedPtr msg);

  /// @brief ROS Subscriber callback function to command a joint trajectory
  /// @param msg JointTrajectoryCommand message dictating the joint(s) to command along with the
  ///   desired trajectory
  /// @details refer to the message definition for details
  void robot_sub_command_traj(const JointTrajectoryCommand::SharedPtr msg);

  /// @brief ROS Service to torque the joints on the robot on/off
  /// @param req TorqueEnable service message request
  /// @param res [out] TorqueEnable service message response [unused]
  /// @details refer to the service definition for details
  bool robot_srv_torque_enable(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<TorqueEnable::Request> req,
    const std::shared_ptr<TorqueEnable::Response> res);

  /// @brief ROS Service to reboot the motors on the robot
  /// @param req Reboot service message request
  /// @param res [out] Reboot service message response [unused]
  /// @details refer to the service definition for details
  bool robot_srv_reboot_motors(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Reboot::Request> req,
    const std::shared_ptr<Reboot::Response> res);

  /// @brief ROS Service that allows the user to get information about the robot
  /// @param req RobotInfo service message request
  /// @param res [out] RobotInfo service message response
  /// @details refer to the service definition for details
  bool robot_srv_get_robot_info(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RobotInfo::Request> req,
    std::shared_ptr<RobotInfo::Response> res);

  /// @brief ROS Service that allows the user to change operating modes
  /// @param req OperatingModes service message request
  /// @param res [out] OperatingModes service message response [unused]
  /// @details refer to the service definition for details
  bool robot_srv_set_operating_modes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<OperatingModes::Request> req,
    const std::shared_ptr<OperatingModes::Response> res);

  /// @brief ROS Service that allows the user to set the motor firmware PID gains
  /// @param req MotorGains service message request
  /// @param res [out] MotorGains service message response [unused]
  /// @details refer to the service defintion for details
  bool robot_srv_set_motor_pid_gains(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<MotorGains::Request> req,
    const std::shared_ptr<MotorGains::Response> res);

  /// @brief ROS Service that allows the user to change a specific register to a specific value for
  ///   multiple motors
  /// @param req RegisterValues service message request
  bool robot_srv_set_motor_registers(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RegisterValues::Request> req,
    const std::shared_ptr<RegisterValues::Response> res);

  /// @brief ROS Service that allows the user to read a specific register on multiple motors
  /// @param req RegisterValues service message request
  /// @param res [out] RegisterValues service message response
  /// @details refer to the service definition for details
  bool robot_srv_get_motor_registers(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<RegisterValues::Request> req,
    std::shared_ptr<RegisterValues::Response> res);

  /// @brief Checks service call requests for validity
  /// @param cmd_type request cmd_type field
  /// @param name request name field
  /// @returns true if the service call request is valid, false otherwise
  /// @details cmd_type must be 'CMD_TYPE_SINGLE' or 'CMD_TYPE_GROUP'; name must be in the
  ///   group_map or motor_map
  bool robot_srv_validate(const std::string & cmd_type, const std::string & name);

  /// @brief ROS One-Shot Timer used to step through a commanded joint trajectory
  void robot_execute_trajectory();

  /// @brief ROS Timer that reads current states from all the motors and publishes them to the
  ///   joint_states topic
  void robot_update_joint_states();
};

}  // namespace interbotix_xs

#endif  // INTERBOTIX_XS_SDK__XS_SDK_OBJ_HPP_
