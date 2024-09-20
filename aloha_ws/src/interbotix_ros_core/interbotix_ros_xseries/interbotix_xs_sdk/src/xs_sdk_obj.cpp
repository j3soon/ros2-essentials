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

#include <string>
#include <vector>
#include <memory>

#include "interbotix_xs_sdk/xs_sdk_obj.hpp"

namespace interbotix_xs
{

InterbotixRobotXS::InterbotixRobotXS(
  bool & success,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("xs_sdk", options)
{
  robot_init_parameters();
  if (!robot_init_driver()) {
    success = false;
    return;
  }
  robot_init_publishers();
  robot_init_subscribers();
  robot_init_services();
  robot_init_timers();
  robot_wait_for_joint_states();
  RCLCPP_INFO(LOGGER, "InterbotixRobotXS is up!");
}

InterbotixRobotXS::~InterbotixRobotXS() {}

bool InterbotixRobotXS::robot_init_driver()
{
  try {
    xs_driver = std::make_unique<InterbotixDriverXS>(
      filepath_motor_configs,
      filepath_mode_configs,
      write_eeprom_on_startup,
      xs_driver_logging_level);
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(LOGGER, "InterbotixDriverXS initialization failed: '%s'.", e.what());
    return false;
  }
  return true;
}

void InterbotixRobotXS::robot_init_parameters()
{
  this->declare_parameter<std::string>("motor_configs", "");
  this->declare_parameter<std::string>("mode_configs", "");
  this->declare_parameter<bool>("load_configs", false);
  this->declare_parameter<std::string>("robot_description", "");
  this->declare_parameter<std::string>("xs_driver_logging_level", "INFO");

  this->get_parameter("motor_configs", filepath_motor_configs);
  this->get_parameter("mode_configs", filepath_mode_configs);
  this->get_parameter("load_configs", write_eeprom_on_startup);
  this->get_parameter("xs_driver_logging_level", xs_driver_logging_level);
  YAML::Node pub_configs = YAML::LoadFile(filepath_motor_configs)["joint_state_publisher"];
  timer_hz = pub_configs["update_rate"].as<int>(100);
  pub_states = pub_configs["publish_states"].as<bool>(true);
  js_topic = pub_configs["topic_name"].as<std::string>("joint_states");
}

void InterbotixRobotXS::robot_init_publishers()
{
  if (pub_states) {
    pub_joint_states = this->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
  }
}

void InterbotixRobotXS::robot_init_subscribers()
{
  using namespace std::placeholders;
  sub_command_group = this->create_subscription<JointGroupCommand>(
    "commands/joint_group",
    10,
    std::bind(&InterbotixRobotXS::robot_sub_command_group, this, _1));
  sub_command_single = this->create_subscription<JointSingleCommand>(
    "commands/joint_single",
    10,
    std::bind(&InterbotixRobotXS::robot_sub_command_single, this, _1));
  sub_command_traj = this->create_subscription<JointTrajectoryCommand>(
    "commands/joint_trajectory",
    10,
    std::bind(&InterbotixRobotXS::robot_sub_command_traj, this, _1));
}

void InterbotixRobotXS::robot_init_services()
{
  using namespace std::placeholders;
  srv_torque_enable = this->create_service<TorqueEnable>(
    "torque_enable",
    std::bind(&InterbotixRobotXS::robot_srv_torque_enable, this, _1, _2, _3));
  srv_reboot_motors = this->create_service<Reboot>(
    "reboot_motors",
    std::bind(&InterbotixRobotXS::robot_srv_reboot_motors, this, _1, _2, _3));
  srv_get_robot_info = this->create_service<RobotInfo>(
    "get_robot_info",
    std::bind(&InterbotixRobotXS::robot_srv_get_robot_info, this, _1, _2, _3));
  srv_operating_modes = this->create_service<OperatingModes>(
    "set_operating_modes",
    std::bind(&InterbotixRobotXS::robot_srv_set_operating_modes, this, _1, _2, _3));
  srv_motor_gains = this->create_service<MotorGains>(
    "set_motor_pid_gains",
    std::bind(&InterbotixRobotXS::robot_srv_set_motor_pid_gains, this, _1, _2, _3));
  srv_set_registers = this->create_service<RegisterValues>(
    "set_motor_registers",
    std::bind(&InterbotixRobotXS::robot_srv_set_motor_registers, this, _1, _2, _3));
  srv_get_registers = this->create_service<RegisterValues>(
    "get_motor_registers",
    std::bind(&InterbotixRobotXS::robot_srv_get_motor_registers, this, _1, _2, _3));
}

void InterbotixRobotXS::robot_init_timers()
{
  execute_joint_traj = false;
  using namespace std::chrono_literals;
  if (timer_hz != 0) {
    // timer that updates the joint states with a period of 1/timer_hz
    tmr_joint_states = this->create_wall_timer(
      std::chrono::nanoseconds(static_cast<int>(1.0e9 / (timer_hz))),
      std::bind(
        &InterbotixRobotXS::robot_update_joint_states,
        this));
  }
}

void InterbotixRobotXS::robot_wait_for_joint_states()
{
  if (timer_hz == 0) {
    return;
  }
  rclcpp::Rate r(10);
  while (rclcpp::ok() && !joint_states.name.size()) {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
  }
}

void InterbotixRobotXS::robot_sub_command_group(const JointGroupCommand::SharedPtr msg)
{
  xs_driver->write_commands(msg->name, msg->cmd);
}

void InterbotixRobotXS::robot_sub_command_single(const JointSingleCommand::SharedPtr msg)
{
  xs_driver->write_joint_command(msg->name, msg->cmd);
}

void InterbotixRobotXS::robot_sub_command_traj(const JointTrajectoryCommand::SharedPtr msg)
{
  using namespace std::chrono_literals;
  if (execute_joint_traj) {
    RCLCPP_WARN(LOGGER, "Trajectory rejected since joints are still moving.");
    return;
  }

  if (msg->traj.points.size() < 2) {
    RCLCPP_WARN(LOGGER, "Trajectory has fewer than 2 points. Aborting...");
    return;
  }

  std::vector<std::string> joint_names;
  if (msg->cmd_type == cmd_type::GROUP) {
    joint_names = xs_driver->get_group_info(msg->name)->joint_names;
  } else if (msg->cmd_type == cmd_type::SINGLE) {
    joint_names.push_back(msg->name);
  }

  if (timer_hz != 0 && msg->traj.points[0].positions.size() == joint_names.size()) {
    for (size_t i{0}; i < joint_names.size(); i++) {
      float expected_state = msg->traj.points[0].positions.at(i);
      float actual_state = joint_states.position.at(xs_driver->get_js_index(joint_names.at(i)));
      if (!(fabs(expected_state - actual_state) < 0.01)) {
        RCLCPP_WARN(
          LOGGER,
          "The %s joint is not at the correct initial state.",
          joint_names.at(i).c_str());
        RCLCPP_WARN(
          LOGGER,
          "Expected state: %.2f, Actual State: %.2f.",
          expected_state, actual_state);
      }
    }
  }
  joint_traj_cmd = msg;
  execute_joint_traj = true;

  // create timer that immediately triggers callback
  tmr_joint_traj = create_wall_timer(
    0s,
    std::bind(
      &InterbotixRobotXS::robot_execute_trajectory,
      this));
}

bool InterbotixRobotXS::robot_srv_torque_enable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<TorqueEnable::Request> req,
  const std::shared_ptr<TorqueEnable::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }

  xs_driver->torque_enable(req->cmd_type, req->name, req->enable);
  return true;
}

bool InterbotixRobotXS::robot_srv_reboot_motors(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Reboot::Request> req,
  const std::shared_ptr<Reboot::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }

  xs_driver->reboot_motors(
    req->cmd_type,
    req->name,
    req->enable,
    req->smart_reboot);
  return true;
}

bool InterbotixRobotXS::robot_srv_get_robot_info(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RobotInfo::Request> req,
  std::shared_ptr<RobotInfo::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }
  bool urdf_exists = false;
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;

  // Parse the urdf model to get joint limit info
  std::string robot_description;
  this->get_parameter("robot_description", robot_description);
  if (!robot_description.empty()) {
    model.initString(robot_description);
    urdf_exists = true;
  }

  // get names, profile type, and operating mode
  if (req->cmd_type == cmd_type::GROUP) {
    res->joint_names = xs_driver->get_group_info(req->name)->joint_names;
    res->profile_type = xs_driver->get_group_info(req->name)->profile_type;
    res->mode = xs_driver->get_group_info(req->name)->mode;
  } else if (req->cmd_type == cmd_type::SINGLE) {
    res->joint_names.push_back(req->name);
    res->profile_type = xs_driver->get_motor_info(req->name)->profile_type;
    res->mode = xs_driver->get_motor_info(req->name)->mode;
  }

  // get the number of joints in the group or single
  res->num_joints = res->joint_names.size();

  for (auto & name : res->joint_names) {
    // get this joint's ID
    res->joint_ids.push_back(xs_driver->get_motor_info(name)->motor_id);
    if (xs_driver->is_motor_gripper(name) > 0) {
      // if the joint is a gripper, add left finger joint info
      res->joint_sleep_positions.push_back(xs_driver->convert_angular_position_to_linear(name, 0));
      // replace gripper joint name with name of left finger joint
      name = xs_driver->get_gripper_info(name)->left_finger;
    } else {
      res->joint_sleep_positions.push_back(xs_driver->get_joint_sleep_position(name));
    }

    // get this joint's joint state index
    res->joint_state_indices.push_back(xs_driver->get_js_index(name));
    if (urdf_exists) {
      // get joint limits if the URDF exists
      ptr = model.getJoint(name);
      res->joint_lower_limits.push_back(ptr->limits->lower);
      res->joint_upper_limits.push_back(ptr->limits->upper);
      res->joint_velocity_limits.push_back(ptr->limits->velocity);
    }
  }
  if (req->name != "all") {
    // if the request name is not "all", return the name from the request
    res->name.push_back(req->name);
  } else {
    // if the request name is "all", return all group names
    for (auto const &[group_name, _] : *xs_driver->get_group_map()) {
      res->name.push_back(group_name);
    }
  }
  return true;
}

bool InterbotixRobotXS::robot_srv_set_operating_modes(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<OperatingModes::Request> req,
  const std::shared_ptr<OperatingModes::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }

  xs_driver->set_operating_modes(
    req->cmd_type,
    req->name,
    req->mode,
    req->profile_type,
    req->profile_velocity,
    req->profile_acceleration);
  return true;
}

bool InterbotixRobotXS::robot_srv_set_motor_pid_gains(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<MotorGains::Request> req,
  const std::shared_ptr<MotorGains::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }

  std::vector<int32_t> gains = {
    req->kp_pos,
    req->ki_pos,
    req->kd_pos,
    req->k1,
    req->k2,
    req->kp_vel,
    req->ki_vel};
  xs_driver->set_motor_pid_gains(req->cmd_type, req->name, gains);
  return true;
}

bool InterbotixRobotXS::robot_srv_set_motor_registers(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RegisterValues::Request> req,
  const std::shared_ptr<RegisterValues::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }

  xs_driver->set_motor_registers(req->cmd_type, req->name, req->reg, req->value);
  return true;
}

bool InterbotixRobotXS::robot_srv_get_motor_registers(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RegisterValues::Request> req,
  std::shared_ptr<RegisterValues::Response> res)
{
  (void)request_header;
  if (!robot_srv_validate(req->cmd_type, req->name)) {
    return false;
  }

  xs_driver->get_motor_registers(req->cmd_type, req->name, req->reg, res->values);
  return true;
}

void InterbotixRobotXS::robot_execute_trajectory()
{
  // get the current real time for this callback execution
  rclcpp::Time current_real = this->get_clock()->now();

  static size_t cntr = 1;
  RCLCPP_DEBUG(
    LOGGER,
    "Executing trajectory step %li/%li.",
    cntr, joint_traj_cmd->traj.points.size());

  // check if end of trajectory has been reached if done, reset counter, set execution status bool
  // to false, and cancel the trajectory execution timer. if not done, cancel the timer (pseudo
  // one-shot), and start another.
  if (cntr == joint_traj_cmd->traj.points.size()) {
    if (!tmr_joint_traj->is_canceled()) {
      tmr_joint_traj->cancel();
    }
    execute_joint_traj = false;
    cntr = 1;
    RCLCPP_DEBUG(LOGGER, "Reached end of trajectory.");
    return;
  } else {
    // cancel trajectory timer
    if (!tmr_joint_traj->is_canceled()) {
      tmr_joint_traj->cancel();
    }

    // get the length of time the timer should be if perfect
    rclcpp::Duration period = std::chrono::nanoseconds(
      joint_traj_cmd->traj.points[cntr].time_from_start.nanosec -
      joint_traj_cmd->traj.points[cntr - 1].time_from_start.nanosec);

    // create new timer with the actual length of time it should execute
    //  (period - (now - start_of_callback))
    tmr_joint_traj = this->create_wall_timer(
      std::chrono::nanoseconds(
        period.nanoseconds() - (
          this->get_clock()->now().nanoseconds() - current_real.nanoseconds())),
      std::bind(
        &InterbotixRobotXS::robot_execute_trajectory,
        this));
  }


  // write commands to the motors depending on cmd_type and mode
  if (joint_traj_cmd->cmd_type == cmd_type::GROUP) {
    // get the mode
    const std::string mode = xs_driver->get_group_info(joint_traj_cmd->name)->mode;
    if (
      (mode == mode::POSITION) ||
      (mode == mode::EXT_POSITION) ||
      (mode == mode::CURRENT_BASED_POSITION) ||
      (mode == mode::LINEAR_POSITION))
    {
      // position commands
      std::vector<float> commands(
        joint_traj_cmd->traj.points[cntr].positions.begin(),
        joint_traj_cmd->traj.points[cntr].positions.end());
      xs_driver->write_commands(joint_traj_cmd->name, commands);
    } else if (mode == mode::VELOCITY) {
      // velocity commands
      std::vector<float> commands(
        joint_traj_cmd->traj.points[cntr].velocities.begin(),
        joint_traj_cmd->traj.points[cntr].velocities.end());
      xs_driver->write_commands(joint_traj_cmd->name, commands);
    } else if ((mode == mode::PWM) || (mode == mode::CURRENT)) {
      // effort commands
      std::vector<float> commands(
        joint_traj_cmd->traj.points[cntr].effort.begin(),
        joint_traj_cmd->traj.points[cntr].effort.end());
      xs_driver->write_commands(joint_traj_cmd->name, commands);
    }
  } else if (joint_traj_cmd->cmd_type == cmd_type::SINGLE) {
    const std::string mode = xs_driver->get_motor_info(joint_traj_cmd->name)->mode;
    if (
      (mode == mode::POSITION) ||
      (mode == mode::EXT_POSITION) ||
      (mode == mode::CURRENT_BASED_POSITION) ||
      (mode == mode::LINEAR_POSITION))
    {
      // position commands
      xs_driver->write_joint_command(
        joint_traj_cmd->name,
        joint_traj_cmd->traj.points[cntr].positions.at(0));
    } else if (mode == mode::VELOCITY) {
      // velocity commands
      xs_driver->write_joint_command(
        joint_traj_cmd->name,
        joint_traj_cmd->traj.points[cntr].velocities.at(0));
    } else if ((mode == mode::PWM) || (mode == mode::CURRENT)) {
      // effort commands
      xs_driver->write_joint_command(
        joint_traj_cmd->name,
        joint_traj_cmd->traj.points[cntr].effort.at(0));
    }
  }
  cntr++;
}

void InterbotixRobotXS::robot_update_joint_states()
{
  sensor_msgs::msg::JointState joint_state_msg;

  joint_state_msg.name = xs_driver->get_all_joint_names();

  xs_driver->get_joint_states(
    "all",
    &joint_state_msg.position,
    &joint_state_msg.velocity,
    &joint_state_msg.effort);
  for (auto const & name : xs_driver->get_gripper_order()) {
    joint_state_msg.name.push_back(xs_driver->get_gripper_info(name)->left_finger.c_str());
    joint_state_msg.name.push_back(xs_driver->get_gripper_info(name)->right_finger.c_str());
    float pos = xs_driver->convert_angular_position_to_linear(
      name, joint_state_msg.position.at(xs_driver->get_gripper_info(name)->js_index));
    joint_state_msg.position.push_back(pos);
    joint_state_msg.position.push_back(-pos);
    joint_state_msg.velocity.push_back(0);
    joint_state_msg.velocity.push_back(0);
    joint_state_msg.effort.push_back(0);
    joint_state_msg.effort.push_back(0);
  }

  // Publish the message to the joint_states topic
  joint_state_msg.header.stamp = this->get_clock()->now();
  joint_states = joint_state_msg;
  if (pub_states) {
    pub_joint_states->publish(joint_state_msg);
  }
}

bool InterbotixRobotXS::robot_srv_validate(
  const std::string & cmd_type,
  const std::string & name)
{
  if (cmd_type == cmd_type::GROUP) {
    if (!xs_driver->get_group_map()->count(name)) {
      RCLCPP_ERROR(
        LOGGER,
        "Group '%s' does not exist. Was it added to the motor config file?",
        name.c_str());
      return false;
    }
  } else if (cmd_type == cmd_type::SINGLE) {
    if (!xs_driver->get_motor_map()->count(name)) {
      RCLCPP_ERROR(
        LOGGER,
        "Joint '%s' does not exist. Was it added to the motor config file?",
        name.c_str());
      return false;
    }
  } else {
    // if command type is invalid, error and return false
    RCLCPP_ERROR(
      LOGGER,
      "cmd_type was '%s'. Choices are 'group' or 'single'.",
      cmd_type.c_str());
    return false;
  }
  return true;
}

}  // namespace interbotix_xs
