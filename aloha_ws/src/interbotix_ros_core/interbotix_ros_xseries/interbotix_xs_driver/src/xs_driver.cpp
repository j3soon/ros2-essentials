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

#include "interbotix_xs_driver/xs_driver.hpp"

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace interbotix_xs
{

InterbotixDriverXS::InterbotixDriverXS(
  std::string filepath_motor_configs,
  std::string filepath_mode_configs,
  bool write_eeprom_on_startup,
  std::string logging_level)
{
  XSLOG_INFO(
    "Using Interbotix X-Series Driver Version: 'v%d.%d.%d'.",
    VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
  logging::set_level(logging_level);
  XSLOG_INFO("Using logging level '%s'.", logging_level.c_str());
  if (!retrieve_motor_configs(filepath_motor_configs, filepath_mode_configs)) {
    throw std::runtime_error("Failed due to bad config.");
  }

  if (!init_port()) {
    throw std::runtime_error("Failed to open port.");
  }

  if (!ping_motors()) {
    XSLOG_FATAL("Failed to find all motors. Shutting down...");
    throw std::runtime_error("Failed to find all motors.");
  }

  if (write_eeprom_on_startup) {
    if (!load_motor_configs()) {
      XSLOG_FATAL("Failed to write configurations to all motors. Shutting down...");
      throw std::runtime_error("Failed to write configurations to all motors.");
    }
    XSLOG_WARN(
      "Writing startup register values to EEPROM. This only needs to be done once on a robot if "
      "using a default motor config file, or after a motor config file has been modified. "
      "Can set `write_eeprom_on_startup` to false from now on.");
  } else {
    XSLOG_INFO("Skipping Load Configs.");
  }

  init_controlItems();
  init_workbench_handlers();
  init_operating_modes();
  init_controlItems();
  XSLOG_INFO("Interbotix X-Series Driver is up!");
}

bool InterbotixDriverXS::set_operating_modes(
  const std::string & cmd_type,
  const std::string & name,
  const std::string & mode,
  const std::string profile_type,
  const int32_t profile_velocity,
  const int32_t profile_acceleration)
{
  if (cmd_type == cmd_type::GROUP && group_map.count(name) > 0) {
    // group case
    for (auto const & joint_name : get_group_info(name)->joint_names) {
      set_joint_operating_mode(
        joint_name,
        mode,
        profile_type,
        profile_velocity,
        profile_acceleration);
    }
    get_group_info(name)->mode = mode;
    get_group_info(name)->profile_type = profile_type;
    get_group_info(name)->profile_velocity = profile_velocity;
    get_group_info(name)->profile_acceleration = profile_acceleration;
    XSLOG_INFO(
      "The operating mode for the '%s' group was changed to '%s' with profile type '%s'.",
      name.c_str(), mode.c_str(), profile_type.c_str());
  } else if (cmd_type == cmd_type::SINGLE && motor_map.count(name) > 0) {
    // single case
    set_joint_operating_mode(
      name,
      mode,
      profile_type,
      profile_velocity,
      profile_acceleration);
    XSLOG_INFO(
      "The operating mode for the '%s' joint was changed to '%s' with profile type '%s'.",
      name.c_str(), mode.c_str(), profile_type.c_str());
  } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
    (cmd_type == cmd_type::GROUP && group_map.count(name) == 0) ||
    (cmd_type == cmd_type::SINGLE && motor_map.count(name) == 0))
  {
    // case where specified joint or group does not exist depending on cmd_type
    XSLOG_ERROR(
      "The '%s' joint/group does not exist. Was it added to the motor config file?",
      name.c_str());
    return false;
  } else {
    // case where cmd_type is invalid (i.e. not 'group' or 'single')
    XSLOG_ERROR("Invalid command for argument 'cmd_type' while setting operating mode.");
    return false;
  }
  return true;
}

bool InterbotixDriverXS::set_joint_operating_mode(
  const std::string & name,
  const std::string & mode,
  const std::string profile_type,
  const int32_t profile_velocity,
  const int32_t profile_acceleration)
{
  for (auto const & motor_name : shadow_map[name]) {
    int32_t drive_mode;
    // read drive mode for each shadow
    dxl_wb.itemRead(motor_map[motor_name].motor_id, "Drive_Mode", &drive_mode);
    std::bitset<8> drive_mode_bitset_read = drive_mode, drive_mode_bitset_write = drive_mode;
    XSLOG_DEBUG(
      "ID: %d, read Drive_Mode [%s].",
      motor_map[motor_name].motor_id, drive_mode_bitset_read.to_string().c_str());
    // The 2nd (0x04) bit of the Drive_Mode register sets Profile Configuration
    // [0/false]: Velocity-based Profile: Create a Profile based on Velocity
    // [1/true]: Time-based Profile: Create Profile based on Time
    if (profile_type == profile::TIME) {
      drive_mode_bitset_write.set(2);  // turn ON for time profile
    } else if (profile_type == profile::VELOCITY) {
      drive_mode_bitset_write.reset(2);  // turn OFF for velocity profile
    }
    // if not correct, write the correct drive mode based on the profile type
    // see https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode for details
    if (!drive_mode_bitset_read.test(2) && profile_type == profile::TIME) {
      // if the 2nd read bit was OFF and profile_type is time, write new Drive_Mode
      for (auto const & joint_name : sister_map[name]) {
        dxl_wb.torque(motor_map[joint_name].motor_id, false);
        XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
      }
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Drive_Mode",
        drive_mode_bitset_write.to_ulong());
      XSLOG_DEBUG(
        "ID: %d, write Drive_Mode [%s].",
        motor_map[motor_name].motor_id, drive_mode_bitset_write.to_string().c_str());
    } else if (drive_mode_bitset_read.test(2) && profile_type == profile::VELOCITY) {
      // if the 2nd read bit was ON and profile_type is velocity, write new Drive_Mode
      for (auto const & joint_name : sister_map[name]) {
        dxl_wb.torque(motor_map[joint_name].motor_id, false);
        XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
      }
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Drive_Mode",
        drive_mode_bitset_write.to_ulong());
      XSLOG_DEBUG(
        "ID: %d, write Drive_Mode [%s].",
        motor_map[motor_name].motor_id, drive_mode_bitset_write.to_string().c_str());
    }

    // Get the present operating mode so we can check against desired operating mode
    // This way, we can hold off on disabling/enabling torque if we are already set
    int32_t opmode;
    dxl_wb.itemRead(motor_map[motor_name].motor_id, "Operating_Mode", &opmode);

    if (mode == mode::POSITION || mode == mode::LINEAR_POSITION) {
      // set position control mode if the desired mode is position or linear_position and not
      // already position
      if (opmode != mode::MODE_POSITION) {
        for (auto const & joint_name : sister_map[name]) {
          dxl_wb.torque(motor_map[joint_name].motor_id, false);
          XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
        }
        dxl_wb.setPositionControlMode(motor_map[motor_name].motor_id);
      } else {
        XSLOG_DEBUG(
          "ID: %d, skipping set mode position.",
          motor_map[motor_name].motor_id);
      }
      // also set prof_acc and prof_vel
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Velocity",
        profile_velocity);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Acceleration",
        profile_acceleration);
      XSLOG_DEBUG(
        "ID: %d, set mode position, prof_vel=%i, prof_acc=%i.",
        motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);
    } else if (mode == mode::EXT_POSITION) {
      // set ext_position control mode if the desired mode is ext_position and not already
      // ext_position
      if (opmode != mode::MODE_EXT_POSITION) {
        for (auto const & joint_name : sister_map[name]) {
          dxl_wb.torque(motor_map[joint_name].motor_id, false);
          XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
        }
        dxl_wb.setExtendedPositionControlMode(motor_map[motor_name].motor_id);
      } else {
        XSLOG_DEBUG(
          "ID: %d, skipping set mode ext_position.",
          motor_map[motor_name].motor_id);
      }
      // set prof_acc and prof_vel
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Velocity",
        profile_velocity);
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Acceleration",
        profile_acceleration);
      XSLOG_DEBUG(
        "ID: %d, set mode ext_position, pv=%i, pa=%i.",
        motor_map[motor_name].motor_id, profile_velocity, profile_acceleration);
    } else if (mode == mode::VELOCITY) {
      // set velocity control mode if the desired mode is velocity and not already velocity
      if (opmode != mode::MODE_VELOCITY) {
        for (auto const & joint_name : sister_map[name]) {
          dxl_wb.torque(motor_map[joint_name].motor_id, false);
          XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
        }
        dxl_wb.setVelocityControlMode(motor_map[motor_name].motor_id);
      } else {
        XSLOG_DEBUG(
          "ID: %d, skipping set mode velocity.",
          motor_map[motor_name].motor_id);
      }
      // also set prof_acc
      dxl_wb.itemWrite(
        motor_map[motor_name].motor_id,
        "Profile_Acceleration",
        profile_acceleration);
      XSLOG_DEBUG(
        "ID: %d, set mode velocity, prof_acc=%i.",
        motor_map[motor_name].motor_id, profile_acceleration);
    } else if (mode == mode::PWM) {
      if (opmode != mode::MODE_PWM) {
        // set pwm control mode if the desired mode is pwm
        for (auto const & joint_name : sister_map[name]) {
          dxl_wb.torque(motor_map[joint_name].motor_id, false);
          XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
        }
        dxl_wb.setPWMControlMode(motor_map[motor_name].motor_id);
      } else {
        XSLOG_DEBUG(
          "ID: %d, skipping set mode pwm.",
          motor_map[motor_name].motor_id);
      }
      XSLOG_DEBUG(
        "ID: %d, set mode pwm.",
        motor_map[motor_name].motor_id);
    } else if (mode == mode::CURRENT) {
      if (opmode != mode::MODE_CURRENT) {
        // set current control mode if the desired mode is current
        for (auto const & joint_name : sister_map[name]) {
          dxl_wb.torque(motor_map[joint_name].motor_id, false);
          XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
        }
        dxl_wb.setCurrentControlMode(motor_map[motor_name].motor_id);
      } else {
        XSLOG_DEBUG(
          "ID: %d, skipping set mode current.",
          motor_map[motor_name].motor_id);
      }
      XSLOG_DEBUG(
        "ID: %d, set mode current.",
        motor_map[motor_name].motor_id);
    } else if (mode == mode::CURRENT_BASED_POSITION) {
      if (opmode != mode::MODE_CURRENT_BASED_POSITION) {
        // set current_based_position control mode if the desired mode is current_based_position
        // and not already current_based_position
        for (auto const & joint_name : sister_map[name]) {
          dxl_wb.torque(motor_map[joint_name].motor_id, false);
          XSLOG_DEBUG("ID: %d, torqued off.", motor_map[joint_name].motor_id);
        }
        dxl_wb.setCurrentBasedPositionControlMode(motor_map[motor_name].motor_id);
      }
      XSLOG_DEBUG(
        "ID: %d, set mode current_based_position.",
        motor_map[motor_name].motor_id);
    } else {
      // mode was invalid
      XSLOG_ERROR(
        "Invalid command for argument 'mode' while setting the operating mode for the %s motor.",
        motor_name.c_str());
      continue;
    }
    // set the new mode and profile_type
    motor_map[motor_name].mode = mode;
    motor_map[motor_name].profile_type = profile_type;
    motor_map[motor_name].profile_velocity = profile_velocity;
    motor_map[motor_name].profile_acceleration = profile_acceleration;
  }

  // torque motors back on
  for (auto const & joint_name : sister_map[name]) {
    dxl_wb.torque(motor_map[joint_name].motor_id, true);
    XSLOG_DEBUG(
      "ID: %d, torqued on.",
      motor_map[joint_name].motor_id);
  }
  return true;
}

bool InterbotixDriverXS::torque_enable(
  const std::string cmd_type,
  const std::string & name,
  const bool & enable)
{
  if (cmd_type == cmd_type::GROUP && group_map.count(name) > 0) {
    // group case
    for (auto const & joint_name : get_group_info(name)->joint_names) {
      // torque each servo in group
      dxl_wb.torque(motor_map[joint_name].motor_id, enable);
    }

    // log torque action
    if (enable) {
      XSLOG_INFO("The '%s' group was torqued on.", name.c_str());
    } else {
      XSLOG_INFO("The '%s' group was torqued off.", name.c_str());
    }
  } else if (cmd_type == cmd_type::SINGLE && motor_map.count(name) > 0) {
    // single case
    // torque the single servo
    dxl_wb.torque(motor_map[name].motor_id, enable);

    // log torque action
    if (enable) {
      XSLOG_INFO("The '%s' joint was torqued on.", name.c_str());
    } else {
      XSLOG_INFO("The '%s' joint was torqued off.", name.c_str());
    }
  } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
    (cmd_type == cmd_type::GROUP && group_map.count(name) == 0) ||
    (cmd_type == cmd_type::SINGLE && motor_map.count(name) == 0))
  {
    // invalid name
    XSLOG_ERROR(
      "The '%s' joint/group does not exist. Was it added to the motor config file?",
      name.c_str());
    return false;
  } else {
    // invalid cmd_type
    XSLOG_ERROR(
      "Invalid command for argument 'cmd_type' while torquing joints.");
    return false;
  }
  return true;
}

bool InterbotixDriverXS::reboot_motors(
  const std::string cmd_type,
  const std::string & name,
  const bool & enable,
  const bool & smart_reboot)
{
  std::vector<std::string> joints_to_torque;
  if (cmd_type == cmd_type::GROUP && group_map.count(name) > 0) {
    // group case
    for (auto const & joint_name : get_group_info(name)->joint_names) {
      // iterate through each joint in group
      if (smart_reboot) {
        // if smart_reboot, find the servos that are in an error status
        int32_t value = 0;
        const char * log;
        bool success = dxl_wb.itemRead(
          motor_map[joint_name].motor_id,
          "Hardware_Error_Status",
          &value, &log);
        if (success && value == 0) {
          continue;
        }
      }
      // reboot the servo
      dxl_wb.reboot(motor_map[joint_name].motor_id);
      XSLOG_INFO("The '%s' joint was rebooted.", joint_name.c_str());
      if (enable) {
        // add servo to joints_to_torque if enabled
        joints_to_torque.push_back(joint_name);
      }
    }
    if (!smart_reboot) {
      XSLOG_INFO("The '%s' group was rebooted.", name.c_str());
    }
  } else if (cmd_type == cmd_type::SINGLE && motor_map.count(name) > 0) {
    // single case
    // reboot the single servo
    dxl_wb.reboot(motor_map[name].motor_id);
    XSLOG_INFO("The '%s' joint was rebooted.", name.c_str());
    if (enable) {
      // add servo to joints_to_torque if enabled
      joints_to_torque.push_back(name);
    }
  } else if ( // NOLINT https://github.com/ament/ament_lint/issues/158
    (cmd_type == cmd_type::GROUP && group_map.count(name) == 0) ||
    (cmd_type == cmd_type::SINGLE && motor_map.count(name) == 0))
  {
    // invalid name
    XSLOG_ERROR(
      "The '%s' joint/group does not exist. Was it added to the motor config file?",
      name.c_str());
    return false;
  } else {
    // invalid cmd_type
    XSLOG_ERROR(
      "Invalid command for argument 'cmd_type' while rebooting motors.");
    return false;
  }

  // torque servos in joints_to_torque and their sisters
  for (const auto & joint_name : joints_to_torque) {
    for (const auto & name : sister_map[joint_name]) {
      torque_enable(cmd_type::SINGLE, name, true);
    }
  }
  return true;
}

template<typename T>
bool InterbotixDriverXS::write_commands(
  const std::string & name,
  std::vector<T> commands_in)
{
  std::vector<float> commands(commands_in.begin(), commands_in.end());
  if (commands.size() != get_group_info(name)->joint_num) {
    XSLOG_ERROR(
      "Number of commands (%ld) does not match the number of joints in group '%s' (%d). "
      "Will not execute.",
      commands.size(), name.c_str(), get_group_info(name)->joint_num);
    return false;
  }
  const std::string mode = get_group_info(name)->mode;
  std::vector<int32_t> dynamixel_commands(commands.size());
  if (
    (mode == mode::POSITION) ||
    (mode == mode::EXT_POSITION) ||
    (mode == mode::CURRENT_BASED_POSITION) ||
    (mode == mode::LINEAR_POSITION))
  {
    // position commands case
    for (size_t i{0}; i < commands.size(); i++) {
      if (mode == mode::LINEAR_POSITION) {
        // convert from linear position if necessary
        commands.at(i) = convert_linear_position_to_radian(
          get_group_info(name)->joint_names.at(i),
          commands.at(i));
      }
      // translate from position to command value
      dynamixel_commands[i] = dxl_wb.convertRadian2Value(
        get_group_info(name)->joint_ids.at(i),
        commands.at(i));
      XSLOG_DEBUG(
        "ID: %d, writing %s command %d.",
        get_group_info(name)->joint_ids.at(i),
        mode.c_str(),
        dynamixel_commands[i]);
    }
    // write position commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
      get_group_info(name)->joint_ids.data(),
      get_group_info(name)->joint_num,
      &dynamixel_commands[0],
      1);
  } else if (mode == mode::VELOCITY) {
    // velocity commands case
    for (size_t i{0}; i < commands.size(); i++) {
      // translate from velocity to command value
      dynamixel_commands[i] = dxl_wb.convertVelocity2Value(
        get_group_info(name)->joint_ids.at(i),
        commands.at(i));
      XSLOG_DEBUG(
        "ID: %d, writing %s command %d.",
        get_group_info(name)->joint_ids.at(i),
        mode.c_str(),
        dynamixel_commands[i]);
    }
    // write velocity commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY,
      get_group_info(name)->joint_ids.data(),
      get_group_info(name)->joint_num,
      &dynamixel_commands[0],
      1);
  } else if (mode == mode::CURRENT) {
    // velocity commands case
    for (size_t i{0}; i < commands.size(); i++) {
      // translate from current to command value
      dynamixel_commands[i] = dxl_wb.convertCurrent2Value(commands.at(i));
      XSLOG_DEBUG(
        "ID: %d, writing %s command %d.",
        get_group_info(name)->joint_ids.at(i),
        mode.c_str(),
        dynamixel_commands[i]);
    }
    // write velocity commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT,
      get_group_info(name)->joint_ids.data(),
      get_group_info(name)->joint_num,
      &dynamixel_commands[0],
      1);
  } else if (mode == mode::PWM) {
    // pwm commands case, don't need to translate from pwm to value
    for (size_t i{0}; i < commands.size(); i++) {
      dynamixel_commands[i] = int32_t(commands.at(i));
      XSLOG_DEBUG(
        "ID: %d, writing %s command %d.",
        get_group_info(name)->joint_ids.at(i),
        mode.c_str(),
        dynamixel_commands[i]);
    }
    // write pwm commands
    dxl_wb.syncWrite(
      SYNC_WRITE_HANDLER_FOR_GOAL_PWM,
      get_group_info(name)->joint_ids.data(),
      get_group_info(name)->joint_num,
      &dynamixel_commands[0],
      1);
  } else {
    // invalid mode
    XSLOG_ERROR(
      "Invalid command for argument 'mode' while commanding joint group.");
    return false;
  }
  return true;
}

bool InterbotixDriverXS::write_position_commands(
  const std::string & name,
  std::vector<float> commands,
  bool blocking)
{
  std::string mode = group_map.at(name).mode;
  if (mode != mode::POSITION) {
    XSLOG_ERROR(
      "Group '%s' is in %s mode not in position mode. Will not execute commands.",
      name.c_str(), mode.c_str());
    return false;
  }

  bool res = write_commands(name, commands);
  if (blocking) {
    std::this_thread::sleep_for(std::chrono::milliseconds(group_map.at(name).profile_velocity));
  }
  return res;
}

bool InterbotixDriverXS::write_joint_command(
  const std::string & name,
  float command)
{
  const std::string mode = motor_map[name].mode;
  if (
    (mode == mode::POSITION) ||
    (mode == mode::EXT_POSITION) ||
    (mode == mode::CURRENT_BASED_POSITION) ||
    (mode == mode::LINEAR_POSITION))
  {
    // position command case
    if (mode == mode::LINEAR_POSITION) {
      // convert from linear position if necessary
      command = convert_linear_position_to_radian(name, command);
    }
    XSLOG_DEBUG(
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write position command
    dxl_wb.goalPosition(motor_map[name].motor_id, command);
  } else if (mode == mode::VELOCITY) {
    // position velocity case
    XSLOG_DEBUG(
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write velocity command
    dxl_wb.goalVelocity(motor_map[name].motor_id, command);
  } else if (mode == mode::CURRENT) {
    // position current case
    XSLOG_DEBUG(
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write current command
    dxl_wb.itemWrite(
      motor_map[name].motor_id, "Goal_Current", dxl_wb.convertCurrent2Value(command));
  } else if (mode == mode::PWM) {
    // pwm current case
    XSLOG_DEBUG(
      "ID: %d, writing %s command %f.",
      motor_map[name].motor_id, mode.c_str(), command);
    // write pwm command
    dxl_wb.itemWrite(motor_map[name].motor_id, "Goal_PWM", int32_t(command));
  } else {
    // invalid mode
    XSLOG_ERROR(
      "Invalid command for argument 'mode' while commanding joint '%s'.", name.c_str());
    return false;
  }
  return true;
}

bool InterbotixDriverXS::set_motor_pid_gains(
  const std::string cmd_type,
  const std::string & name,
  const std::vector<int32_t> & gains)
{
  std::vector<std::string> names;
  if (cmd_type == cmd_type::GROUP) {
    // group case, get names from group map
    names = get_group_info(name)->joint_names;
  } else if (cmd_type == cmd_type::SINGLE) {
    // single case, just use given name
    names.push_back(name);
  }

  for (auto const & name : names) {
    // write gains for each servo
    uint8_t id = motor_map[name].motor_id;
    XSLOG_DEBUG("ID: %d, writing gains:", motor_map[name].motor_id);
    XSLOG_DEBUG("        Pos_P: %i", gains.at(0));
    XSLOG_DEBUG("        Pos_I: %i", gains.at(1));
    XSLOG_DEBUG("        Pos_D: %i", gains.at(2));
    XSLOG_DEBUG("        FF_1: %i", gains.at(3));
    XSLOG_DEBUG("        FF_2: %i", gains.at(4));
    XSLOG_DEBUG("        Vel_P: %i", gains.at(5));
    XSLOG_DEBUG("        Vel_I: %i", gains.at(6));
    dxl_wb.itemWrite(id, "Position_P_Gain", gains.at(0));
    dxl_wb.itemWrite(id, "Position_I_Gain", gains.at(1));
    dxl_wb.itemWrite(id, "Position_D_Gain", gains.at(2));
    dxl_wb.itemWrite(id, "Feedforward_1st_Gain", gains.at(3));
    dxl_wb.itemWrite(id, "Feedforward_2nd_Gain", gains.at(4));
    dxl_wb.itemWrite(id, "Velocity_P_Gain", gains.at(5));
    dxl_wb.itemWrite(id, "Velocity_I_Gain", gains.at(6));
  }
  return true;
}

bool InterbotixDriverXS::set_motor_registers(
  const std::string cmd_type,
  const std::string & name,
  const std::string & reg,
  const int32_t & value)
{
  std::vector<std::string> names;
  if (cmd_type == cmd_type::GROUP) {
    // group case, get names from group map
    names = get_group_info(name)->joint_names;
  } else if (cmd_type == cmd_type::SINGLE) {
    // single case, just use given name
    names.push_back(name);
  }

  for (auto const & name : names) {
    // write register for each servo
    XSLOG_DEBUG(
      "ID: %d, writing reg: %s, value: %d.",
      motor_map[name].motor_id, reg.c_str(), value);
    dxl_wb.itemWrite(motor_map[name].motor_id, reg.c_str(), value);
  }
  return true;
}

bool InterbotixDriverXS::get_motor_registers(
  const std::string cmd_type,
  const std::string & name,
  const std::string & reg,
  std::vector<int32_t> & values)
{
  std::vector<std::string> names;
  if (cmd_type == cmd_type::GROUP) {
    // group case, get names from group map
    names = get_group_info(name)->joint_names;
  } else if (cmd_type == cmd_type::SINGLE) {
    // single case, just use given name
    names.push_back(name);
  }

  // get info on the register that is going to be read from
  const ControlItem * goal_reg = dxl_wb.getItemInfo(
    motor_map.at(names.front()).motor_id, reg.c_str());
  if (goal_reg == NULL) {
    XSLOG_ERROR(
      "Could not get '%s' Item Info. Did you spell the register name correctly?",
      reg.c_str());
    return false;
  }

  for (auto const & name : names) {
    int32_t value = 0;
    const char * log;
    // read register for each servo and check result for success
    if (!dxl_wb.itemRead(motor_map[name].motor_id, reg.c_str(), &value, &log)) {
      XSLOG_ERROR("%s", log);
      return false;
    } else {
      XSLOG_DEBUG(
        "ID: %d, reading reg: '%s', value: %d.", motor_map[name].motor_id, reg.c_str(), value);
    }

    // add register to values vector with type depending on data_length
    if (goal_reg->data_length == 1) {
      values.push_back((uint8_t)value);
    } else if (goal_reg->data_length == 2) {
      values.push_back((int16_t)value);
    } else {
      values.push_back(value);
    }
  }
  return true;
}

bool InterbotixDriverXS::get_joint_states(
  const std::string & name,
  std::vector<float> * positions,
  std::vector<float> * velocities,
  std::vector<float> * effort)
{
  read_joint_states();
  std::lock_guard<std::mutex> guard(_mutex_js);
  for (const auto & joint_name : get_group_info(name)->joint_names) {
    // iterate through each joint in group, reading pos, vel, and eff
    if (positions) {
      positions->push_back(robot_positions.at(get_js_index(joint_name)));
    }
    if (velocities) {
      velocities->push_back(robot_velocities.at(get_js_index(joint_name)));
    }
    if (effort) {
      effort->push_back(robot_efforts.at(get_js_index(joint_name)));
    }
  }
  return true;
}

bool InterbotixDriverXS::get_joint_states(
  const std::string & name,
  std::vector<double> * positions,
  std::vector<double> * velocities,
  std::vector<double> * effort)
{
  read_joint_states();
  std::lock_guard<std::mutex> guard(_mutex_js);
  for (const auto & joint_name : get_group_info(name)->joint_names) {
    // iterate through each joint in group, reading pos, vel, and eff
    if (positions) {
      positions->push_back(robot_positions.at(get_js_index(joint_name)));
    }
    if (velocities) {
      velocities->push_back(robot_velocities.at(get_js_index(joint_name)));
    }
    if (effort) {
      effort->push_back(robot_efforts.at(get_js_index(joint_name)));
    }
  }
  return true;
}

bool InterbotixDriverXS::get_joint_state(
  const std::string & name,
  float * position,
  float * velocity,
  float * effort)
{
  read_joint_states();
  std::lock_guard<std::mutex> guard(_mutex_js);
  // read pos, vel, and eff for specified joint
  if (position) {
    *position = robot_positions.at(get_js_index(name));
  }
  if (velocity) {
    *velocity = robot_velocities.at(get_js_index(name));
  }
  if (effort) {
    *effort = robot_efforts.at(get_js_index(name));
  }
  return true;
}

float InterbotixDriverXS::convert_linear_position_to_radian(
  const std::string & name,
  const float & linear_position)
{
  float half_dist = linear_position / 2.0;
  float arm_length = gripper_map[name].arm_length;
  float horn_radius = gripper_map[name].horn_radius;

  // (pi / 2) - acos(horn_rad^2 + (pos / 2)^2 - arm_length^2) / (2 * horn_rad * (pos / 2))
  return 3.14159 / 2.0 - \
         acos(
    (pow(horn_radius, 2) + \
    pow(half_dist, 2) - \
    pow(arm_length, 2)) / (2 * horn_radius * half_dist));
}

float InterbotixDriverXS::convert_angular_position_to_linear(
  const std::string & name,
  const float & angular_position)
{
  float arm_length = gripper_map[name].arm_length;
  float horn_radius = gripper_map[name].horn_radius;
  float a1 = horn_radius * sin(angular_position);
  float c = sqrt(pow(horn_radius, 2) - pow(a1, 2));
  float a2 = sqrt(pow(arm_length, 2) - pow(c, 2));
  return a1 + a2;
}

bool InterbotixDriverXS::retrieve_motor_configs(
  std::string filepath_motor_configs,
  std::string filepath_mode_configs)
{
  // read motor_configs param
  XSLOG_INFO("Retrieving motor config file at '%s'.", filepath_motor_configs.c_str());
  try {
    // try to load motor_configs yaml file
    motor_configs = YAML::LoadFile(filepath_motor_configs.c_str());
  } catch (YAML::BadFile & error) {
    // if file is not found or a bad format, shut down
    XSLOG_FATAL("Motor Config file was not found or has a bad format. Shutting down...");
    XSLOG_FATAL("YAML Error: '%s'", error.what());
    return false;
  }

  if (motor_configs.IsNull()) {
    // if motor_configs is not found or empty, shut down
    XSLOG_FATAL("Motor Config file was not found. Shutting down...");
    return false;
  }

  // read mode_configs param
  XSLOG_INFO("Retrieving mode config file at '%s'.", filepath_mode_configs.c_str());
  try {
    // try to load mode_configs yaml file
    mode_configs = YAML::LoadFile(filepath_mode_configs.c_str());
    XSLOG_INFO(
      "Loaded mode configs from '%s'.",
      filepath_mode_configs.c_str());
  } catch (YAML::BadFile & error) {
    // if file is not found or a bad format, shut down
    XSLOG_FATAL(
      "Mode Config file was not found or has a bad format. Shutting down...");
    XSLOG_FATAL(
      "YAML Error: '%s'",
      error.what());
    return false;
  }

  if (mode_configs.IsNull()) {
    // if mode_configs is not found or empty, use defaults
    XSLOG_WARN("Mode Config file is empty. Will use defaults.");
  }

  // use specified or default port
  port = motor_configs["port"].as<std::string>(DEFAULT_PORT);
  if (mode_configs["port"]) {
    port = mode_configs["port"].as<std::string>(DEFAULT_PORT);
  }

  // create all_motors node from 'motors'
  YAML::Node all_motors = motor_configs["motors"];
  for (
    YAML::const_iterator motor_itr = all_motors.begin();
    motor_itr != all_motors.end();
    motor_itr++)
  {
    // iterate through each motor in all_motors node
    // get motor name
    std::string motor_name = motor_itr->first.as<std::string>();
    // create single_motor node from the motor_name
    YAML::Node single_motor = all_motors[motor_name];
    // extract ID from node
    uint8_t id = (uint8_t)single_motor["ID"].as<int32_t>();
    // add the motor to the motor_map with it's ID, pos as the default opmode, vel as default
    //  profile
    motor_map.insert({motor_name, {id, mode::POSITION, profile::VELOCITY}});
    for (
      YAML::const_iterator info_itr = single_motor.begin();
      info_itr != single_motor.end();
      info_itr++)
    {
      // iterate through the single_motor node
      // save all registers that are not ID or Baud_Rate
      std::string reg = info_itr->first.as<std::string>();
      if (reg != "ID" && reg != "Baud_Rate") {
        int32_t value = info_itr->second.as<int32_t>();
        MotorRegVal motor_info = {id, reg, value};
        motor_info_vec.push_back(motor_info);
      }
    }
  }

  // create all_grippers node from 'grippers'
  YAML::Node all_grippers = motor_configs["grippers"];
  for (
    YAML::const_iterator gripper_itr = all_grippers.begin();
    gripper_itr != all_grippers.end();
    gripper_itr++)
  {
    // iterate through each gripper in all_grippers node
    // get gripper name
    std::string gripper_name = gripper_itr->first.as<std::string>();
    // create single_gripper node from the gripper_name
    YAML::Node single_gripper = all_grippers[gripper_name];
    // initialize a Gripper struct to save the info about this gripper
    Gripper gripper;
    // load all info from the single_gripper node into the Griper struct, substituting the default
    //  values if not given the value
    gripper.horn_radius = single_gripper["horn_radius"].as<float>(0.014);
    gripper.arm_length = single_gripper["arm_length"].as<float>(0.024);
    gripper.left_finger = single_gripper["left_finger"].as<std::string>("left_finger");
    gripper.right_finger = single_gripper["right_finger"].as<std::string>("right_finger");
    gripper_map.insert({gripper_name, gripper});
  }

  // create joint_order node from 'joint_order'
  YAML::Node joint_order = motor_configs["joint_order"];
  // create sleep_positions node from 'sleep_positions'
  YAML::Node sleep_positions = motor_configs["sleep_positions"];
  // create JointGroup struct for all_joints - contains all joints in robot
  JointGroup all_joints;
  all_joints.joint_num = (uint8_t) joint_order.size();
  all_joints.mode = mode::POSITION;
  all_joints.profile_type = profile::VELOCITY;
  all_joints.profile_velocity = DEFAULT_PROF_VEL;
  all_joints.profile_acceleration = DEFAULT_PROF_ACC;
  for (size_t i{0}; i < joint_order.size(); i++) {
    // iterate through each joint in joint_order list
    // save each joint ID and name
    std::string joint_name = joint_order[i].as<std::string>();
    all_joints.joint_names.push_back(joint_name);
    all_joints.joint_ids.push_back(motor_map[joint_name].motor_id);
    // add this joint's index to the js_index_map
    js_index_map.insert({joint_name, i});
    // add any shadows
    shadow_map.insert({joint_name, {joint_name}});
    // add any sisters
    sister_map.insert({joint_name, {joint_name}});
    // add the sleep position of this joint, defaults to 0 if not specified
    sleep_map.insert({joint_name, sleep_positions[i].as<float>(0)});
    // if this joint is a gripper, add it to the gripper_name
    if (gripper_map.count(joint_name) > 0) {
      gripper_map[joint_name].js_index = i;
      gripper_order.push_back(joint_name);
    }
  }

  // append the left and right finger to the gripper_map
  for (auto const & name : gripper_order) {
    js_index_map.insert({gripper_map[name].left_finger, js_index_map.size()});
    js_index_map.insert({gripper_map[name].right_finger, js_index_map.size()});
  }

  // all the all_joints JointGroup to the group_map
  group_map.insert({"all", all_joints});
  // create a pointer from the group_map's all group reference
  all_ptr = &group_map.at("all");

  // create all_shadows node from 'shadows'
  YAML::Node all_shadows = motor_configs["shadows"];
  for (
    YAML::const_iterator master_itr = all_shadows.begin();
    master_itr != all_shadows.end();
    master_itr++)
  {
    // iterate through each shadow servo's master in all_shadows node
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    YAML::Node shadow_list = master["shadow_list"];
    for (size_t i{0}; i < shadow_list.size(); i++) {
      shadow_map[master_name].push_back(shadow_list[i].as<std::string>());
    }
  }

  // create all_sisters node from 'sisters'
  YAML::Node all_sisters = motor_configs["sisters"];
  for (
    YAML::const_iterator sister_itr = all_sisters.begin();
    sister_itr != all_sisters.end();
    sister_itr++)
  {
    // iterate through each sister servo in all_sisters node
    // save each 2-in-1 servo to the sister_map
    std::string sister_one = sister_itr->first.as<std::string>();
    std::string sister_two = sister_itr->second.as<std::string>();
    sister_map[sister_one].push_back(sister_two);
    sister_map[sister_two].push_back(sister_one);
  }

  // create all_groups node from 'groups'
  YAML::Node all_groups = motor_configs["groups"];
  for (
    YAML::const_iterator group_itr = all_groups.begin();
    group_itr != all_groups.end();
    group_itr++)
  {
    // iterate through each sister servo in all_sisters node
    // get the name of the group
    std::string name = group_itr->first.as<std::string>();
    // get the list of joints in the group from the all_group map
    YAML::Node joint_list = all_groups[name];
    // create a JointGroup for this group
    JointGroup group;
    // the number of joints in this group is the size of the list of joints
    group.joint_num = (uint8_t) joint_list.size();
    for (size_t i{0}; i < joint_list.size(); i++) {
      // add each joint's name and id to the JointGroup
      std::string joint_name = joint_list[i].as<std::string>();
      group.joint_names.push_back(joint_name);
      group.joint_ids.push_back(motor_map[joint_name].motor_id);
    }
    // add the JointGroup and its name to the group_map
    group_map.insert({name, group});
  }

  YAML::Node pub_configs = motor_configs["joint_state_publisher"];
  read_failure_behavior = static_cast<ReadFailureBehavior>(
    pub_configs["read_failure_behavior"].as<int>(ReadFailureBehavior::PUB_DXL_WB));

  // Do input validation on read failure behavior parameter
  if (
    read_failure_behavior < ReadFailureBehavior::PUB_DXL_WB ||
    read_failure_behavior > ReadFailureBehavior::PUB_NAN)
  {
    // If out of range or invalid, default to PUB_DXL_WB/0
    XSLOG_ERROR(
      "Invalid option %d provided to joint_state_publisher.read_failure_behavior. "
      "Will default to option 0.",
      read_failure_behavior);
    read_failure_behavior = ReadFailureBehavior::PUB_DXL_WB;
  }
  XSLOG_DEBUG("read_failure_behavior set to %d.", read_failure_behavior);

  XSLOG_INFO(
    "Loaded motor configs from '%s'.",
    filepath_motor_configs.c_str());
  return true;
}

bool InterbotixDriverXS::init_port()
{
  // try to connect to the specified port at the default baudrate
  if (!dxl_wb.init(port.c_str(), DEFAULT_BAUDRATE)) {
    // if the connection fails, shut down
    XSLOG_FATAL(
      "Failed to open port at '%s'. Shutting down...",
      port.c_str());
    return false;
  }
  return true;
}

bool InterbotixDriverXS::ping_motors()
{
  // result is true by default, if can't find motor, set this to false
  bool found_all_motors = true;
  const char * log;
  for (size_t cntr_ping_motors = 0; cntr_ping_motors < 3; cntr_ping_motors++) {
    XSLOG_INFO(
      "Pinging all motors specified in the motor_config file. (Attempt %ld/3)",
      cntr_ping_motors + 1);
    // iterate through each servo in the motor_map
    for (const auto &[motor_name, motor_state] : motor_map) {
      // try to ping the servo
      if (!dxl_wb.ping(motor_state.motor_id, &log)) {
        // if any ping is unsuccessful, shut down
        XSLOG_ERROR(
          "\tCan't find DYNAMIXEL ID: %2.d, Joint Name: '%s':\n\t\t  '%s'",
          motor_state.motor_id, motor_name.c_str(), log);
        found_all_motors = false;
      } else {
        XSLOG_INFO(
          "\tFound DYNAMIXEL ID: %2.d, Model: '%s', Joint Name: '%s'.",
          motor_state.motor_id, dxl_wb.getModelName(motor_state.motor_id), motor_name.c_str());
      }
      // untorque each pinged servo, need to write data to the EEPROM
      dxl_wb.torque(motor_state.motor_id, false);
    }
    if (found_all_motors) {
      return found_all_motors;
    }
  }
  return found_all_motors;
}

bool InterbotixDriverXS::load_motor_configs()
{
  // result is true by default, if can't write config, set this to false
  bool wrote_all_configs = true;
  for (auto const & motor_info : motor_info_vec) {
    if (!dxl_wb.itemWrite(motor_info.motor_id, motor_info.reg.c_str(), motor_info.value)) {
      XSLOG_FATAL(
        "Failed to write value[%d] on items[%s] to [ID : %2.d]",
        motor_info.value, motor_info.reg.c_str(), motor_info.motor_id);
      wrote_all_configs = false;
    }
  }
  return wrote_all_configs;
}

bool InterbotixDriverXS::init_controlItems()
{
  uint8_t motor_id = motor_map.begin()->second.motor_id;

  const ControlItem * goal_position = dxl_wb.getItemInfo(motor_id, "Goal_Position");
  if (!goal_position) {
    XSLOG_ERROR("Could not get 'Goal_Position' Item Info");
    return false;
  }

  const ControlItem * goal_velocity = dxl_wb.getItemInfo(motor_id, "Goal_Velocity");
  if (!goal_velocity) {
    goal_velocity = dxl_wb.getItemInfo(motor_id, "Moving_Speed");
  }
  if (!goal_velocity) {
    XSLOG_ERROR("Could not get 'Goal_Velocity' or 'Moving_Speed' Item Info");
    return false;
  }

  const ControlItem * goal_current = NULL;
  for (auto const & [_, motor_info] : motor_map) {
    goal_current = dxl_wb.getItemInfo(motor_info.motor_id, "Goal_Current");
    if (goal_current) {
      break;
    }
  }

  if (!goal_current) {
    XSLOG_WARN(
      "Could not get 'Goal_Current' Item Info. This message can be "
      "ignored if none of the robot's motors support current control.");
  }

  const ControlItem * goal_pwm = dxl_wb.getItemInfo(motor_id, "Goal_PWM");
  if (!goal_pwm) {
    XSLOG_ERROR("Could not get 'Goal_PWM' Item Info");
    return false;
  }

  const ControlItem * present_position = dxl_wb.getItemInfo(motor_id, "Present_Position");
  if (!present_position) {
    XSLOG_ERROR("Could not get 'Present_Position' Item Info");
    return false;
  }

  const ControlItem * present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Velocity");
  if (!present_velocity) {
    present_velocity = dxl_wb.getItemInfo(motor_id, "Present_Speed");
  }
  if (!present_velocity) {
    XSLOG_ERROR("Could not get 'Present_Velocity' or 'Present_Speed' Item Info");
    return false;
  }

  const ControlItem * present_current = dxl_wb.getItemInfo(motor_id, "Present_Current");
  if (!present_current) {
    present_current = dxl_wb.getItemInfo(motor_id, "Present_Load");
  }
  if (!present_current) {
    XSLOG_FATAL("Could not get 'Present_Current' or 'Present_Load' Item Info");
    return false;
  }

  control_items["Goal_Position"] = goal_position;
  control_items["Goal_Velocity"] = goal_velocity;
  control_items["Goal_Current"] = goal_current;
  control_items["Goal_PWM"] = goal_pwm;
  control_items["Present_Position"] = present_position;
  control_items["Present_Velocity"] = present_velocity;
  control_items["Present_Current"] = present_current;
  return true;
}

bool InterbotixDriverXS::init_workbench_handlers()
{
  if (
    !dxl_wb.addSyncWriteHandler(
      control_items["Goal_Position"]->address, control_items["Goal_Position"]->data_length))
  {
    XSLOG_FATAL("Failed to add SyncWriteHandler for Goal_Position.");
    return false;
  }

  if (
    !dxl_wb.addSyncWriteHandler(
      control_items["Goal_Velocity"]->address, control_items["Goal_Velocity"]->data_length))
  {
    XSLOG_FATAL("Failed to add SyncWriteHandler for Goal_Velocity.");
    return false;
  }

  // only add a SyncWriteHandler for 'Goal_Current' if the register actually exists!
  if (control_items["Goal_Current"]) {
    if (
      !dxl_wb.addSyncWriteHandler(
        control_items["Goal_Current"]->address, control_items["Goal_Current"]->data_length))
    {
      XSLOG_FATAL("Failed to add SyncWriteHandler for Goal_Current.");
      return false;
    }
  } else {
    XSLOG_WARN("SyncWriteHandler for Goal_Current not added as it's not supported.");
  }

  if (
    !dxl_wb.addSyncWriteHandler(
      control_items["Goal_PWM"]->address, control_items["Goal_PWM"]->data_length))
  {
    XSLOG_FATAL("Failed to add SyncWriteHandler for Goal_PWM.");
    return false;
  }

  if (dxl_wb.getProtocolVersion() == 2.0f) {
    uint16_t start_address = std::min(
      control_items["Present_Position"]->address,
      control_items["Present_Current"]->address);
    /*
      As some models have an empty space between Present_Velocity and Present Current, read_length
      is modified as below.
    */
    // uint16_t read_length = control_items["Present_Position"]->data_length \
    //   + control_items["Present_Velocity"]->data_length \
    //   + control_items["Present_Current"]->data_length;
    uint16_t read_length = control_items["Present_Position"]->data_length + \
      control_items["Present_Velocity"]->data_length + \
      control_items["Present_Current"]->data_length + \
      2;
    if (!dxl_wb.addSyncReadHandler(start_address, read_length)) {
      XSLOG_FATAL("Failed to add SyncReadHandler.");
      return false;
    }
  }
  return true;
}

void InterbotixDriverXS::init_operating_modes()
{
  YAML::Node all_shadows = motor_configs["shadows"];
  for (
    YAML::const_iterator master_itr = all_shadows.begin();
    master_itr != all_shadows.end();
    master_itr++)
  {
    std::string master_name = master_itr->first.as<std::string>();
    YAML::Node master = all_shadows[master_name];
    if (master["calibrate"].as<bool>(false)) {
      int32_t master_position;
      dxl_wb.itemRead(motor_map[master_name].motor_id, "Present_Position", &master_position);
      for (auto const & shadow_name : shadow_map[master_name]) {
        if (shadow_name == master_name) {
          continue;
        }
        XSLOG_DEBUG(
          "Calibrating motor pair with IDs %d, %d.",
          motor_map[master_name].motor_id,
          motor_map[shadow_name].motor_id);
        dxl_wb.itemWrite(motor_map[shadow_name].motor_id, "Homing_Offset", 0);
        int32_t shadow_position, shadow_drive_mode;
        dxl_wb.itemRead(motor_map[shadow_name].motor_id, "Present_Position", &shadow_position);
        dxl_wb.itemRead(motor_map[shadow_name].motor_id, "Drive_Mode", &shadow_drive_mode);
        // The 0th (0x01) bit of the Drive_Mode register sets Normal/Reverse Mode
        // [0/false]: Normal Mode: CCW(Positive), CW(Negative)
        // [1/true]: Reverse Mode: CCW(Negative), CW(Positive)
        // This mode dictates how to calculate the homing offset of the shadow motor
        int32_t homing_offset;
        if (static_cast<std::bitset<8>>(shadow_drive_mode).test(0)) {
          homing_offset = shadow_position - master_position;
        } else {
          homing_offset = master_position - shadow_position;
        }
        dxl_wb.itemWrite(motor_map[shadow_name].motor_id, "Homing_Offset", homing_offset);
      }
    }
  }

  YAML::Node all_groups = mode_configs["groups"];
  for (
    YAML::const_iterator group_itr = all_groups.begin();
    group_itr != all_groups.end();
    group_itr++)
  {
    std::string name = group_itr->first.as<std::string>();
    YAML::Node single_group = all_groups[name];
    const std::string operating_mode =
      single_group["operating_mode"].as<std::string>(DEFAULT_OP_MODE);
    const std::string profile_type =
      single_group["profile_type"].as<std::string>(DEFAULT_PROF_TYPE);
    int32_t profile_velocity = single_group["profile_velocity"].as<int32_t>(DEFAULT_PROF_VEL);
    int32_t profile_acceleration =
      single_group["profile_acceleration"].as<int32_t>(DEFAULT_PROF_ACC);
    set_operating_modes(
      cmd_type::GROUP,
      name,
      operating_mode,
      profile_type,
      profile_velocity,
      profile_acceleration);
    if (!single_group["torque_enable"].as<bool>(TORQUE_ENABLE)) {
      torque_enable(cmd_type::GROUP, name, false);
    }
  }

  YAML::Node all_singles = mode_configs["singles"];
  for (
    YAML::const_iterator single_itr = all_singles.begin();
    single_itr != all_singles.end();
    single_itr++)
  {
    std::string single_name = single_itr->first.as<std::string>();
    YAML::Node single_joint = all_singles[single_name];
    const std::string operating_mode =
      single_joint["operating_mode"].as<std::string>(DEFAULT_OP_MODE);
    const std::string profile_type =
      single_joint["profile_type"].as<std::string>(DEFAULT_PROF_TYPE);
    int32_t profile_velocity = single_joint["profile_velocity"].as<int32_t>(DEFAULT_PROF_VEL);
    int32_t profile_acceleration =
      single_joint["profile_acceleration"].as<int32_t>(DEFAULT_PROF_ACC);
    set_operating_modes(
      cmd_type::SINGLE,
      single_name,
      operating_mode,
      profile_type,
      profile_velocity,
      profile_acceleration);
    if (!single_joint["torque_enable"].as<bool>(TORQUE_ENABLE)) {
      torque_enable(cmd_type::SINGLE, single_name, false);
    }
  }
}

void InterbotixDriverXS::read_joint_states()
{
  std::lock_guard<std::mutex> guard(_mutex_js);
  robot_positions.clear();
  robot_velocities.clear();
  robot_efforts.clear();
  const char * log;

  std::vector<int32_t> get_current(all_ptr->joint_num, 0);
  std::vector<int32_t> get_velocity(all_ptr->joint_num, 0);
  std::vector<int32_t> get_position(all_ptr->joint_num, 0);

  bool read_failed = false;
  if (dxl_wb.getProtocolVersion() == 2.0f) {
    // Execute sync read from all pinged DYNAMIXELs
    if (!dxl_wb.syncRead(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        &log))
    {
      XSLOG_ERROR("Failed syncRead: %s", log);
      read_failed = true;
    }

    // Gets present current of all servos
    if (!dxl_wb.getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        control_items["Present_Current"]->address,
        control_items["Present_Current"]->data_length,
        get_current.data(),
        &log))
    {
      XSLOG_ERROR("Failed getSyncReadData for Present_Current: %s", log);
      read_failed = true;
    }

    // Gets present velocity of all servos
    if (!dxl_wb.getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        control_items["Present_Velocity"]->address,
        control_items["Present_Velocity"]->data_length,
        get_velocity.data(),
        &log))
    {
      XSLOG_ERROR("Failed getSyncReadData for Present_Velocity: %s", log);
      read_failed = true;
    }

    // Gets present position of all servos
    if (!dxl_wb.getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        all_ptr->joint_ids.data(),
        all_ptr->joint_num,
        control_items["Present_Position"]->address,
        control_items["Present_Position"]->data_length,
        get_position.data(),
        &log))
    {
      XSLOG_ERROR("Failed getSyncReadData for Present_Position: %s", log);
      read_failed = true;
    }

    uint8_t index = 0;
    for (auto const & id : all_ptr->joint_ids) {
      float position = 0.0;
      float velocity = 0.0;
      float effort = 0.0;

      if (strcmp(dxl_wb.getModelName(id), "XL-320") == 0) {
        effort = dxl_wb.convertValue2Load(get_current.at(index));
      } else {
        effort = dxl_wb.convertValue2Current(get_current.at(index));
      }
      velocity = dxl_wb.convertValue2Velocity(id, get_velocity.at(index));
      position = dxl_wb.convertValue2Radian(id, get_position.at(index));
      robot_efforts.push_back(effort);
      robot_velocities.push_back(velocity);
      robot_positions.push_back(position);
      index++;
    }
  } else if (dxl_wb.getProtocolVersion() == 1.0f) {
    uint16_t length_of_data = control_items["Present_Position"]->data_length +
      control_items["Present_Velocity"]->data_length +
      control_items["Present_Current"]->data_length;
    std::vector<uint32_t> get_all_data(length_of_data, 0);

    for (auto const & id : all_ptr->joint_ids) {
      if (!dxl_wb.readRegister(
          id,
          control_items["Present_Position"]->address,
          length_of_data,
          get_all_data.data(),
          &log))
      {
        XSLOG_ERROR("Failed readRegister for joint states: %s", log);
        read_failed = true;
      }

      int16_t effort_raw = DXL_MAKEWORD(get_all_data.at(4), get_all_data.at(5));
      int32_t velocity_raw = DXL_MAKEWORD(get_all_data.at(2), get_all_data.at(3));
      int32_t position_raw = DXL_MAKEWORD(get_all_data.at(0), get_all_data.at(1));

      // Convert raw register values to the metric system
      float effort = dxl_wb.convertValue2Load(effort_raw);
      float velocity = dxl_wb.convertValue2Velocity(id, velocity_raw);
      float position = dxl_wb.convertValue2Radian(id, position_raw);

      robot_efforts.push_back(effort);
      robot_velocities.push_back(velocity);
      robot_positions.push_back(position);
    }
  }

  // If read failed, check to see what motors we can actually read from. This will provide
  // some additional troubleshooting information on what motors may have been disconnected or are
  // unresponsive
  if (read_failed) {
    // Note that this process is slow and dramatically reduces the joint state pub frequency
    // However, we are in a failure state so performance does not matter
    for (const auto id : all_ptr->joint_ids) {
      int32_t value = 0;
      // Try to read from an item available on all motor models
      if (!dxl_wb.itemRead(id, "ID", &value)) {
        // Log the motor IDs we can't read from
        XSLOG_ERROR("[xs_sdk] Failed to read from DYNAMIXEL ID: %d", id);
      }
    }
    // If read failed and SDK is configured to publish NaNs, fill the joint state message with NaNs
    if (read_failure_behavior == ReadFailureBehavior::PUB_NAN) {
      const auto nan_states = std::vector<float>(
        robot_positions.size(),
        std::numeric_limits<float>::quiet_NaN());
      robot_positions = std::vector<float>(nan_states);
      robot_velocities = std::vector<float>(nan_states);
      robot_efforts = std::vector<float>(nan_states);
    }
  }
}

std::vector<std::string> InterbotixDriverXS::get_all_joint_names()
{
  return all_ptr->joint_names;
}

float InterbotixDriverXS::get_joint_sleep_position(const std::string & name)
{
  return sleep_map[name];
}

std::unordered_map<std::string, JointGroup> * InterbotixDriverXS::get_group_map()
{
  return &group_map;
}

std::unordered_map<std::string, MotorState> * InterbotixDriverXS::get_motor_map()
{
  return &motor_map;
}

JointGroup * InterbotixDriverXS::get_group_info(const std::string & name)
{
  return &group_map.at(name);
}

MotorState * InterbotixDriverXS::get_motor_info(const std::string & name)
{
  return &motor_map.at(name);
}

Gripper * InterbotixDriverXS::get_gripper_info(const std::string & name)
{
  return &gripper_map.at(name);
}

std::vector<std::string> InterbotixDriverXS::get_gripper_order()
{
  return gripper_order;
}

size_t InterbotixDriverXS::get_js_index(const std::string & name)
{
  return js_index_map.at(name);
}

bool InterbotixDriverXS::is_motor_gripper(const std::string & name)
{
  return gripper_map.count(name) > 0;
}

bool InterbotixDriverXS::go_to_sleep_configuration(
  const std::string & name,
  const bool blocking)
{
  XSLOG_DEBUG("Sending robot to sleep configuration.");
  std::vector<float> sleep_positions;
  for (auto const & joint_name : get_group_info(name)->joint_names) {
    sleep_positions.push_back(get_joint_sleep_position(joint_name));
  }
  return write_position_commands(name, sleep_positions, blocking);
}

bool InterbotixDriverXS::go_to_home_configuration(
  const std::string & name,
  const bool blocking)
{
  XSLOG_DEBUG("Sending robot to home configuration.");
  std::vector<float> home_positions(get_group_info(name)->joint_num, 0.0);
  return write_position_commands(name, home_positions, blocking);
}

}  // namespace interbotix_xs
