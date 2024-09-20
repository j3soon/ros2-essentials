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

#include "interbotix_slate_driver/slate_base.hpp"

#include <chrono>

namespace slate_base
{

SlateBase::SlateBase(const rclcpp::NodeOptions & options)
: rclcpp::Node("slate_base", "", options), cmd_vel_x_(0.0), cmd_vel_z_(0.0), cnt_(0), x_(0.0),
  y_(0.0), theta_(0.0), x_vel_(0.0), z_omega_(0.0), is_first_odom_(true), pose_{0},
  right_motor_c_(0.0), left_motor_c_(0.0), chassis_state_(SystemState::SYS_INIT),
  publish_tf_(false), max_vel_x_(1.0), max_vel_z_(1.0), current_time_(get_clock()->now()),
  last_time_(get_clock()->now()), tf_broadcaster_odom_(this),
  cmd_vel_time_last_update_(get_clock()->now()),
  cmd_vel_timeout_(rclcpp::Duration(std::chrono::milliseconds(CMD_TIME_OUT)))
{
  using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;

  declare_parameter<bool>("publish_tf", false);
  declare_parameter<std::string>("odom_frame_name", "odom");
  declare_parameter<std::string>("base_frame_name", "base_link");

  get_parameter("publish_tf", publish_tf_);
  get_parameter("odom_frame_name", odom_frame_name_);
  get_parameter("base_frame_name", base_frame_name_);

  pub_odom_ = create_publisher<Odometry>("odom", 50);
  pub_battery_state_ = create_publisher<BatteryState>("battery_state", 1);

  sub_cmd_vel_ = create_subscription<Twist>(
    "cmd_vel",
    1,
    std::bind(&SlateBase::cmd_vel_callback, this, _1));

  srv_set_text_ = create_service<SetString>(
    "set_text",
    std::bind(&SlateBase::set_text_callback, this, _1, _2, _3));

  srv_motor_torque_status_ = create_service<SetBool>(
    "set_motor_torque_status",
    std::bind(&SlateBase::motor_torque_status_callback, this, _1, _2, _3));

  srv_enable_charging_ = create_service<SetBool>(
    "enable_charging",
    std::bind(&SlateBase::enable_charging_callback, this, _1, _2, _3));

  srv_set_light_state_ = create_service<SetLightState>(
    "set_light_state",
    std::bind(&SlateBase::set_light_state_callback, this, _1, _2, _3));

  std::string dev;
  if (!base_driver::chassisInit(dev)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize base port.");
    ::exit(EXIT_FAILURE);
  }
  RCLCPP_INFO(get_logger(), "Initalized base at port '%s'.", dev.c_str());
  char version[32] = {0};
  if (base_driver::getVersion(version)) {
    RCLCPP_INFO(get_logger(), "Base version: %s", version);
  }
}

void SlateBase::update()
{
  rclcpp::spin_some(get_node_base_interface());
  current_time_ = get_clock()->now();

  // time out velocity commands
  if (current_time_ - cmd_vel_time_last_update_ > cmd_vel_timeout_) {
    cmd_vel_x_ = 0.0f;
    cmd_vel_z_ = 0.0f;
  }

  // limit velocity commands
  cmd_vel_x_ = std::min(max_vel_x_, std::max(-max_vel_x_, cmd_vel_x_));
  cmd_vel_z_ = std::min(max_vel_z_, std::max(-max_vel_z_, cmd_vel_z_));

  // initialize chassis data and use it to update the driver
  base_driver::ChassisData data = {
    .cmd_vel_x=cmd_vel_x_,
    .cmd_vel_y=0.0,
    .cmd_vel_z=cmd_vel_z_,
    .light_state=light_state_,
    .system_state=0};

  if (!base_driver::updateChassisInfo(&data)) {
    return;
  }

  // extract and update base system command bytes
  sys_cmd_ = data.cmd;

  // update battery state every 10 iterations
  cnt_++;
  auto battery_state = BatteryState();
  if (cnt_ % 10 == 0) {
    battery_state.header.stamp = current_time_;
    battery_state.voltage = data.voltage;
    battery_state.current = data.current;
    battery_state.percentage = data.charge;
    battery_state.power_supply_status = data.system_state;
    pub_battery_state_->publish(battery_state);
  }

  // update odometry values
  x_vel_ = data.vel_x;
  z_omega_ = data.vel_z;

  x_ = data.odom_x;
  y_ = data.odom_y;
  theta_ = data.odom_z;

  if (is_first_odom_) {
    pose_[0] = x_;
    pose_[1] = y_;
    pose_[2] = theta_;
    is_first_odom_ = false;
  }

  // create transform
  tf2::Quaternion q;
  q.setRPY(0, 0, wrap_angle(theta_ - pose_[2]));
  auto odom_quat = tf2::toMsg(q);
  auto odom_trans = TransformStamped();
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_name_;
  odom_trans.child_frame_id = base_frame_name_;

  odom_trans.transform.translation.x =
    cos(-pose_[2]) * (x_ - pose_[0]) - sin(-pose_[2]) * (y_ - pose_[1]);
  odom_trans.transform.translation.y =
    sin(-pose_[2]) * (x_ - pose_[0]) + cos(-pose_[2]) * (y_ - pose_[1]);
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  if (publish_tf_) {
    tf_broadcaster_odom_.sendTransform(odom_trans);
  }

  // publish odometry
  auto odom = Odometry();
  odom.header.stamp = current_time_;
  odom.header.frame_id = odom_frame_name_;

  // set position
  odom.pose.pose.position.x = odom_trans.transform.translation.x;
  odom.pose.pose.position.y = odom_trans.transform.translation.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance[0] = (data.system_state == SystemState::SYS_ESTOP) ? -1 : 1;

  // set velocity
  odom.child_frame_id = base_frame_name_;
  odom.twist.twist.linear.x = x_vel_;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = z_omega_;

  pub_odom_->publish(odom);
  last_time_ = current_time_;
}

void SlateBase::cmd_vel_callback(const Twist::SharedPtr msg)
{
  cmd_vel_x_ = msg->linear.x;
  cmd_vel_z_ = msg->angular.z;
  cmd_vel_time_last_update_ = get_clock()->now();
}

bool SlateBase::set_text_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetString::Request> req,
  const std::shared_ptr<SetString::Response> res)
{
  res->message = "Set base screen text to: '" + req->data + "'.";
  base_driver::setText(req->data.c_str());
  RCLCPP_INFO(get_logger(), res->message.c_str());
  res->success = true;
  return true;
}

bool SlateBase::motor_torque_status_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetBool::Request> req,
  const std::shared_ptr<SetBool::Response> res)
{
  req->data ? sys_cmd_ &= ~(1): sys_cmd_ |= 1;
  res->success = base_driver::setSysCmd(sys_cmd_);

  std::string enabled_disabled = req->data ? "enable" : "disable";
  if (res->success) {
    res->message = "Successfully " + enabled_disabled + "d motor torque.";
    RCLCPP_INFO(get_logger(), res->message.c_str());
  } else {
    res->message = "Failed to " + enabled_disabled + " motor torque.";
    RCLCPP_ERROR(get_logger(), res->message.c_str());
  }
  return res->success;
}

bool SlateBase::enable_charging_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetBool::Request> req,
  const std::shared_ptr<SetBool::Response> res)
{
  req->data ? sys_cmd_ |= 2 : sys_cmd_ &= ~(2);
  res->success = base_driver::setSysCmd(sys_cmd_);

  std::string enabled_disabled = req->data ? "enable" : "disable";
  if (res->success) {
    res->message = "Successfully " + enabled_disabled + "d charging.";
    RCLCPP_INFO(get_logger(), res->message.c_str());
  } else {
    res->message = "Failed to " + enabled_disabled + " charging.";
    RCLCPP_ERROR(get_logger(), res->message.c_str());
  }
  return res->success;
}

bool SlateBase::set_light_state_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<SetLightState::Request> req,
  const std::shared_ptr<SetLightState::Response> res)
{
  res->message = "Set light state to: '" + std::to_string(req->light_state) + "'.";
  light_state_ = req->light_state;
  RCLCPP_INFO(get_logger(), res->message.c_str());
  res->success = true;
  return true;
}

float SlateBase::wrap_angle(float angle)
{
  if (angle > M_PI) {
    angle = angle - 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle = angle + 2.0 * M_PI;
  }
  return angle;
}

}  // namespace slate_base
