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

#ifndef INTERBOTIX_XS_DRIVER__XS_COMMON_HPP_
#define INTERBOTIX_XS_DRIVER__XS_COMMON_HPP_

#include "interbotix_xs_driver/xs_logging.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace interbotix_xs
{

// All motors are preset to 1M baud
#define DEFAULT_BAUDRATE 1000000

// Udev rule creates a symlink with this name
#define DEFAULT_PORT "/dev/ttyDXL"

// Write goal positions [rad] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0

// Write goal velocities [rad/s] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// Write goal currents [mA] to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 2

// Write goal pwms to multiple motors at the same time
#define SYNC_WRITE_HANDLER_FOR_GOAL_PWM 3

// Read current joint states for multiple motors at the same time
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// Default motor operating mode is 'position'
inline static const std::string DEFAULT_OP_MODE = "position";

// Default motor profile type is 'velocity' (as opposed to 'time')
inline static const std::string DEFAULT_PROF_TYPE = "velocity";

// Allow joint velocity to be infinite when in position control mode - makes robot very reactive to
// joint commands
static const int32_t DEFAULT_PROF_VEL = 0;

// Allow joint acceleration to be infinite when in position control mode - makes robot very
// reactive to joint commands
static const int32_t DEFAULT_PROF_ACC = 0;

// Torque motor on by default
static const bool TORQUE_ENABLE = true;

// Get motor configurations by default
static bool LOAD_CONFIGS = true;

namespace mode
{

// Constants for operating modes
inline static const std::string PWM = "pwm";
inline static const std::string POSITION = "position";
inline static const std::string EXT_POSITION = "ext_position";
inline static const std::string CURRENT_BASED_POSITION = "current_based_position";
inline static const std::string LINEAR_POSITION = "linear_position";
inline static const std::string VELOCITY = "velocity";
inline static const std::string CURRENT = "current";

inline static const int MODE_PWM = 16;
inline static const int MODE_POSITION = 3;
inline static const int MODE_EXT_POSITION = 4;
inline static const int MODE_CURRENT_BASED_POSITION = 5;
inline static const int MODE_VELOCITY = 1;
inline static const int MODE_CURRENT = 0;

}  // namespace mode

namespace cmd_type
{

// constants for command types
inline static const std::string GROUP = "group";
inline static const std::string SINGLE = "single";

}  // namespace cmd_type

namespace profile
{

// constants for profile types
inline static const std::string VELOCITY = "velocity";
inline static const std::string TIME = "time";

}  // profile


// Struct to hold multiple joints that represent a group
struct JointGroup
{
  // Names of all joints in the group
  std::vector<std::string> joint_names;
  // Dynamixel ID of all joints in the group
  std::vector<uint8_t> joint_ids;
  // Number of joints in the group
  uint8_t joint_num;
  // Operating Mode for all joints in the group
  std::string mode;
  // Profile Type ('velocity' or 'time') for all joints in the group
  std::string profile_type;
  // Profile Velocity (in ms) for all joints in the group
  int32_t profile_velocity;
  // Profile Acceleration (in ms) for all joints in the group
  int32_t profile_acceleration;
};

// Struct to hold data on a single motor
struct MotorState
{
  // Dynamixel ID of the motor
  uint8_t motor_id;
  // Operating Mode of the motor
  std::string mode;
  // Profile Type ('velocity' or 'time') for the motor
  std::string profile_type;
  // Profile Velocity (in ms) for the motor
  int32_t profile_velocity;
  // Profile Acceleration (in ms) for the motor
  int32_t profile_acceleration;
};

// Struct to hold data on an Interbotix Gripper
struct Gripper
{
  // Index in the published JointState message 'name' list belonging to the gripper motor
  size_t js_index;
  // Distance [m] from the motor horn's center to its edge
  float horn_radius;
  // Distance [m] from the edge of the motor horn to a finger
  float arm_length;
  // Name of the 'left_finger' joint as defined in the URDF (if present)
  std::string left_finger;
  // Name of the 'right_finger' joint as defined in the URDF (if present)
  std::string right_finger;
};

// Struct to hold a desired register value for a given motor
struct MotorRegVal
{
  // Dynamixel ID of a motor
  uint8_t motor_id;
  // Register name
  std::string reg;
  // Value to write to the above register for the specified motor
  int32_t value;
};

using MapGroup = std::unordered_map<std::string, JointGroup>;
using MapMotor = std::unordered_map<std::string, MotorState>;
using MapGripper = std::unordered_map<std::string, Gripper>;

// Enum defining the behavior on joint state read failures
enum ReadFailureBehavior
{
  // Publishes whatever is given from the dxl_wb for all joints states (-pi)
  PUB_DXL_WB = 0,
  // Publishes NaN values for all joint states
  PUB_NAN = 1
};

}  // namespace interbotix_xs

#endif  // INTERBOTIX_XS_DRIVER__XS_COMMON_HPP_
