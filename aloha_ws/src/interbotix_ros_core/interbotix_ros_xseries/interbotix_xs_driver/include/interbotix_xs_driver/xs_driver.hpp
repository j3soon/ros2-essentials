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

#ifndef INTERBOTIX_XS_DRIVER__XS_DRIVER_HPP_
#define INTERBOTIX_XS_DRIVER__XS_DRIVER_HPP_

#include <cmath>
#include <chrono>
#include <string>
#include <bitset>
#include <vector>
#include <algorithm>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "interbotix_xs_driver/version.hpp"
#include "interbotix_xs_driver/xs_common.hpp"
#include "interbotix_xs_driver/xs_logging.hpp"

namespace interbotix_xs
{

// Interbotix Core Class to build any type of DYNAMIXEL-based robot
class InterbotixDriverXS
{
public:
  /// @brief Constructor for the InterbotixDriverXS
  /// @param filepath_motor_configs absolute filepath to the motor configuration file
  /// @param filepath_mode_configs absolute filepath to the modes configuration file
  /// @param write_eeprom_on_startup whether or not to write configs to EEPROM on startup,
  ///   defaults to `true`
  /// @param logging_level the logging level of the xs_logging utility; defaults to "INFO"
  /// @throws runtime_error if constructor was not initialized properly due to bad configuration
  ///   file, inability to open specified serial port, inability to ping all specified motors, or
  ///   inability to write startup configuration to all motors' EEPROMs
  explicit InterbotixDriverXS(
    std::string filepath_motor_configs,
    std::string filepath_mode_configs,
    bool write_eeprom_on_startup = true,
    std::string logging_level = "INFO");

  /// @brief Destructor for the InterbotixDriverXS
  ~InterbotixDriverXS() {}

  /// @brief Set the operating mode for a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if changing the operating mode for a group of motors
  ///   or 'CMD_TYPE_SINGLE' if changing the operating mode for a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param mode desired operating mode (either 'position', 'linear_position', 'ext_position',
  ///   'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based
  ///   Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration passthrough to the Profile_Acceleration register on the motor
  bool set_operating_modes(
    const std::string & cmd_type,
    const std::string & name,
    const std::string & mode,
    const std::string profile_type = DEFAULT_PROF_TYPE,
    const int32_t profile_velocity = DEFAULT_PROF_VEL,
    const int32_t profile_acceleration = DEFAULT_PROF_ACC);

  /// @brief Set the operating mode for a single motor
  /// @param name desired motor name
  /// @param mode desired operating mode (either 'position', 'linear_position', 'ext_position',
  ///   'velocity', 'pwm', 'current', or 'current_based_position')
  /// @param profile_type set to 'velocity' for a Velocity-based Profile or 'time' for a Time-based
  ///   Profile (modifies Bit 2 of the 'Drive_Mode' register)
  /// @param profile_velocity passthrough to the Profile_Velocity register on the motor
  /// @param profile_acceleration passthrough to the Profile_Acceleration register on the motor
  bool set_joint_operating_mode(
    const std::string & name,
    const std::string & mode,
    const std::string profile_type = DEFAULT_PROF_TYPE,
    const int32_t profile_velocity = DEFAULT_PROF_VEL,
    const int32_t profile_acceleration = DEFAULT_PROF_ACC);

  /// @brief Torque On/Off a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if torquing off a group of motors or
  ///   'CMD_TYPE_SINGLE' if torquing off a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param enable set to True to torque on or False to torque off
  bool torque_enable(
    const std::string cmd_type,
    const std::string & name,
    const bool & enable);

  /// @brief Reboot a specific group of motors or a single motor
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if rebooting a group of motors or 'CMD_TYPE_SINGLE'
  ///   if rebooting a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param torque_enable set to True to torque on or False to torque off after rebooting
  /// @param smart_reboot set to True to only reboot motor(s) in a specified group that have gone
  ///   into an error state
  bool reboot_motors(
    const std::string cmd_type,
    const std::string & name,
    const bool & enable,
    const bool & smart_reboot);

  /// @brief Command a desired group of motors with the specified commands
  /// @param name desired motor group name
  /// @param commands vector of commands (order matches the order specified in the 'groups'
  ///   section in the motor config file)
  /// @details commands are processed differently based on the operating mode specified for the
  ///   motor group
  template<typename T>
  bool write_commands(
    const std::string & name,
    std::vector<T> commands);

  /// @brief Command a desired group of motors with the specified position commands
  /// @param name desired motor group name
  /// @param commands vector of commands (order matches the order specified in the 'groups'
  ///   section in the motor config file)
  /// @param blocking whether the function should wait to return control to the user until the
  ///   robot finishes moving; set to false by default
  /// @details will not perform movement if desired group is not in position mode
  bool write_position_commands(
    const std::string & name,
    std::vector<float> commands,
    bool blocking = false);

  /// @brief Command a desired motor with the specified command
  /// @param name desired motor name
  /// @param command motor command
  /// @details the command is processed differently based on the operating mode specified for the
  ///   motor
  bool write_joint_command(
    const std::string & name,
    float command);

  /// @brief Set motor firmware PID gains
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if changing the PID gains for a group of motors or
  ///   'CMD_TYPE_SINGLE' if changing the PID gains for a single motor
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param gains vector containing the desired PID gains - order is as shown in the function
  bool set_motor_pid_gains(
    const std::string cmd_type,
    const std::string & name,
    const std::vector<int32_t> & gains);

  /// @brief Set a register value to multiple motors
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if setting register values for a group of motors or
  ///   'CMD_TYPE_SINGLE' if setting a single register value
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param reg the register to set
  /// @param value desired register value
  bool set_motor_registers(
    const std::string cmd_type,
    const std::string & name,
    const std::string & reg,
    const int32_t & value);

  /// @brief Get a register value from multiple motors
  /// @param cmd_type set to 'CMD_TYPE_GROUP' if getting register values from a group of motors or
  ///   'CMD_TYPE_SINGLE' if getting a single register value
  /// @param name desired motor group name if cmd_type is set to 'CMD_TYPE_GROUP' or the desired
  ///   motor name if cmd_type is set to 'CMD_TYPE_SINGLE'
  /// @param reg the register to get
  /// @param values [out] vector of register values
  bool get_motor_registers(
    const std::string cmd_type,
    const std::string & name,
    const std::string & reg,
    std::vector<int32_t> & values);

  /// @brief Get states for a group of joints
  /// @param name desired joint group name
  /// @param positions [out] vector of current joint positions [rad]
  /// @param velocities [out] vector of current joint velocities [rad/s]
  /// @param effort [out] vector of current joint effort [mA]
  bool get_joint_states(
    const std::string & name = "all",
    std::vector<float> * positions = NULL,
    std::vector<float> * velocities = NULL,
    std::vector<float> * effort = NULL);

  /// @brief Get states for a group of joints
  /// @param name desired joint group name
  /// @param positions [out] vector of current joint positions [rad]
  /// @param velocities [out] vector of current joint velocities [rad/s]
  /// @param effort [out] vector of current joint effort [mA]
  bool get_joint_states(
    const std::string & name = "all",
    std::vector<double> * positions = NULL,
    std::vector<double> * velocities = NULL,
    std::vector<double> * effort = NULL);

  /// @brief Get states for a single joint
  /// @param name desired joint name
  /// @param position [out] current joint position [rad]
  /// @param velocity [out] current joint velocity [rad/s]
  /// @param effort [out] current joint effort [mA]
  bool get_joint_state(
    const std::string & name,
    float * position = NULL,
    float * velocity = NULL,
    float * effort = NULL);

  /// @brief Get states for a single joint
  /// @param name desired joint name
  /// @param position [out] current joint position [rad]
  /// @param velocity [out] current joint velocity [rad/s]
  /// @param effort [out] current joint effort [mA]
  bool get_joint_state(
    const std::string & name,
    double * position = NULL,
    double * velocity = NULL,
    double * effort = NULL);

  /// @brief Convert linear distance between two gripper fingers into angular position
  /// @param name name of the gripper servo to command
  /// @param linear_position desired distance [m] between the two gripper fingers
  /// @returns angular position [rad] that achieves the desired linear distance
  float convert_linear_position_to_radian(
    const std::string & name,
    const float & linear_position);

  /// @brief Convert angular position into the linear distance from one gripper finger to the
  ///   center of the gripper servo horn
  /// @param name name of the gripper servo to command
  /// @param angular_position desired gripper angular position [rad]
  /// @returns linear position [m] from a gripper finger to the center of the gripper servo horn
  float convert_angular_position_to_linear(
    const std::string & name,
    const float & angular_position);

  /// @brief Get a vector of all joint names in the all_ptr JointGroup
  std::vector<std::string> get_all_joint_names();

  /// @brief Get the sleep position of a given joint
  /// @param name the name of the joint to get the sleep position of
  /// @returns the sleep position of the given joint
  float get_joint_sleep_position(const std::string & name);

  /// @brief Get the group map
  /// @returns pointer to the group map
  std::unordered_map<std::string, JointGroup> * get_group_map();

  /// @brief Get the motor map
  /// @returns pointer to the motor map
  std::unordered_map<std::string, MotorState> * get_motor_map();

  /// @brief Get info on the given joint group
  /// @param name name of the joint group to get info on
  /// @returns pointer to JointGroup
  JointGroup * get_group_info(const std::string & name);

  /// @brief Get MotorState of given joint
  /// @param name name of the joint to get motor info on
  /// @returns pointer to MotorState
  MotorState * get_motor_info(const std::string & name);

  /// @brief Get MotorState of given gripper
  /// @param name name of the gripper to get info on
  /// @returns pointer to Gripper
  Gripper * get_gripper_info(const std::string & name);

  /// @brief Get ordered vector that determines the order in which gripper joint states are
  ///   retrieved
  /// @returns gripper order vector
  std::vector<std::string> get_gripper_order();

  /// @brief Get the joint state index of a joint
  /// @param name the name of the motor to get the js index of
  /// @returns the js index of the given joint
  size_t get_js_index(const std::string & name);

  /// @brief Determine if given joint is a gripper
  /// @param name name of the joint
  /// @returns true if a gripper, false otherwise
  bool is_motor_gripper(const std::string & name);

  /// @brief Send the specified joint group to its sleep configuration
  /// @param name group name to send to sleep configuration; defaults to 'all'
  /// @param blocking: (optional) whether the function should wait to return control to the user
  ///   until the robot finishes moving; false by default
  bool go_to_sleep_configuration(
    const std::string & name,
    const bool blocking = false);

  /// @brief Send the specified joint group to its home configuration
  /// @param name group name to send to home configuration; defaults to 'all'
  /// @param blocking: (optional) whether the function should wait to return control to the user
  ///   until the robot finishes moving; false by default
  bool go_to_home_configuration(
    const std::string & name,
    const bool blocking = false);

private:
  // Pointer to the 'all' group (makes updating joint states more efficient)
  JointGroup * all_ptr;

  // DynamixelWorkbench object used to easily communicate with any DYNAMIXEL servo
  DynamixelWorkbench dxl_wb;

  // Holds all the information in the motor_configs YAML file
  YAML::Node motor_configs;

  // Holds all the information in the mode_configs YAML file
  YAML::Node mode_configs;

  // Holds the USB port name that connects to the U2D2
  std::string port = DEFAULT_PORT;

  // Vector containing all the desired EEPROM register values to command the motors at startup
  std::vector<MotorRegVal> motor_info_vec;

  // Vector containing the order in which multiple grippers (if present) are published in the
  // JointState message
  std::vector<std::string> gripper_order;

  // Dictionary mapping register names to information about them (like 'address' and expected 'data
  // length')
  std::unordered_map<std::string, const ControlItem *> control_items;

  // Dictionary mapping a joint's name with its 'sleep' position
  std::unordered_map<std::string, float> sleep_map;

  // Dictionary mapping a joint group's name with information about it (as defined in the
  // JointGroup struct)
  std::unordered_map<std::string, JointGroup> group_map;

  // Dictionary mapping a motor's name with information about it (as defined in the MotorState
  // struct)
  std::unordered_map<std::string, MotorState> motor_map;

  // Dictionary mapping a motor's name with the names of its shadows - including itself
  std::unordered_map<std::string, std::vector<std::string>> shadow_map;

  // Dictionary mapping the name of either servo in a 2in1 motor with the other one (henceforth
  // known as 'sister')
  std::unordered_map<std::string, std::vector<std::string>> sister_map;

  // Dictionary mapping the name of a gripper motor with information about it (as defined in the
  // Gripper struct)
  std::unordered_map<std::string, Gripper> gripper_map;

  // Dictionary mapping the name of a joint with its position in the JointState 'name' list
  std::unordered_map<std::string, size_t> js_index_map;

  // Vector containing the robot joint positions in [rad] at the last update
  std::vector<float> robot_positions;

  // Vector containing the robot joint velocities in [rad/s] at the last update
  std::vector<float> robot_velocities;

  // Vector containing the robot joint efforts in [mA] at the last update
  std::vector<float> robot_efforts;

  /// Absolute filepath to the motor configs file
  std::string filepath_motor_configs;

  /// Absolute filepath to the mode configs file
  std::string filepath_mode_configs;

  /// true if the robot should write to the all servos' EEPROMs on startup, false otherwise
  bool write_eeprom_on_startup;

  // Mutex for updating / getting joint states
  std::mutex _mutex_js;

  // The behavior on joint state read failures
  ReadFailureBehavior read_failure_behavior{ReadFailureBehavior::PUB_DXL_WB};

  /// @brief Loads a robot-specific 'motor_configs' yaml file and populates class variables with
  ///   its contents
  bool retrieve_motor_configs(
    std::string filepath_motor_configs,
    std::string filepath_mode_configs);

  /// @brief Initializes the port to communicate with the DYNAMIXEL servos
  /// @returns True if the port was successfully opened; False otherwise
  bool init_port();

  /// @brief Pings all motors to make sure they can be found
  /// @returns True if all motors were found; False otherwise
  bool ping_motors();

  /// @brief Writes some 'startup' EEPROM register values to the DYNAMIXEL servos
  /// @returns True if all register values were written successfully; False otherwise
  bool load_motor_configs();

  /// @brief Retrieves information about 'Goal_XXX' and 'Present_XXX' registers
  /// @returns true if succeeded to init control items, false otherwise
  /// @details Info includes a register's name, address, and data length
  bool init_controlItems();

  /// @brief Creates SyncWrite and SyncRead Handlers to write/read data to multiple motors
  ///   simultaneously
  /// @returns true if succeeded to init workbench handlers, false otherwise
  bool init_workbench_handlers();

  /// @brief Loads a 'mode_configs' yaml file containing desired operating modes and sets up the
  ///   motors accordingly
  void init_operating_modes();

  /// @brief Updates the joint states from the DYNAMIXELs
  void read_joint_states();
};

}  // namespace interbotix_xs

#endif  // INTERBOTIX_XS_DRIVER__XS_DRIVER_HPP_
