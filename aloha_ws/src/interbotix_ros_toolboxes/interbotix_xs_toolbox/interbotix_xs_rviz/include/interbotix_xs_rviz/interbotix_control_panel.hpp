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

#ifndef INTERBOTIX_XS_RVIZ__INTERBOTIX_CONTROL_PANEL_HPP_
#define INTERBOTIX_XS_RVIZ__INTERBOTIX_CONTROL_PANEL_HPP_

#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#endif

#include "rviz_common/panel.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "QCompleter"
#include "QLineEdit"
#include "QPushButton"

#include "interbotix_xs_msgs/srv/reboot.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/register_values.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_rviz/xs_register_descriptions.hpp"

namespace Ui
{
class InterbotixControlPanelUI;
}  // namespace Ui

namespace interbotix_xs_rviz
{

inline static const std::string CMD_TYPE_GROUP = "group";
inline static const std::string CMD_TYPE_SINGLE = "single";
inline static const std::string NAME_ALL = "all";
inline static const std::string SELECT = "Select:";
inline static const std::regex ns_regex(
  "^\\/([a-zA-Z0-9_]+)\\/",
  std::regex_constants::ECMAScript);

static const rclcpp::Logger LOGGER = rclcpp::get_logger(
  "interbotix_xs_rviz.interbotix_control_panel");

using TorqueEnable = interbotix_xs_msgs::srv::TorqueEnable;
using RobotInfo = interbotix_xs_msgs::srv::RobotInfo;
using RegisterValues = interbotix_xs_msgs::srv::RegisterValues;
using Reboot = interbotix_xs_msgs::srv::Reboot;
using OperatingModes = interbotix_xs_msgs::srv::OperatingModes;

using JointGroupCommand = interbotix_xs_msgs::msg::JointGroupCommand;


class InterbotixControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit InterbotixControlPanel(QWidget * parent = 0);
  ~InterbotixControlPanel();

  void onInitialize() override;
  void onEnable();
  void onDisable();

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  bool set_robot_namespace(const QString & robot_namespace);


  /** -------- Torque Tab ---------------- */

  void send_torque_enable_call(bool enable);


  /** -------- Torque Tab ---------------- */

  void send_home_sleep_call();


  /** -------- Reboot Tab ---------------- */

  void send_reboot_call();


  /** -------- Operating Modes Tab ------- */

  void send_opmodes_call();


  /** -------- Get Register Values Tab --- */

  void send_getregval_call();

protected Q_SLOTS:
  void update_robot_namespace();
  void update_robot_info();


  /** -------- Torque Tab ---------------- */

  void torque_change_cmd_type();
  void torque_change_name();
  void torque_enable_torque();
  void torque_disable_torque();
  void torque_init();


  /** -------- Home/Sleep Tab ------------ */

  void homesleep_go_to_home();
  void homesleep_go_to_sleep();
  void homesleep_init();


  /** -------- Reboot Tab ---------------- */

  void reboot_change_cmd_type();
  void reboot_change_name();
  void reboot_change_smartreboot(bool checked);
  void reboot_change_enable(bool checked);
  void reboot_init();


  /** -------- Operating Modes Tab ------- */

  void opmodes_change_cmd_type();
  void opmodes_change_name();
  void opmodes_change_mode(int);
  void opmodes_change_profile_type(int);
  void opmodes_change_profile_vel();
  void opmodes_change_profile_acc();
  void opmodes_init();


  /** -------- Get Register Values Tab --- */

  void getregval_change_cmd_type();
  void getregval_change_name();
  void getregval_change_reg_name(int);
  void getregval_display(const RegisterValues::Response::SharedPtr &);
  void getregval_init();


  /** -------- Emergency Stop Tab -------- */

  void estop_button_pressed();

protected:
  // The GUI for this RViz panel
  Ui::InterbotixControlPanelUI * ui_;

  // This node is a pointer to the node run by RViz (unused)
  rclcpp::Node::SharedPtr node_;

  // This node will be spun to use publishers and service calls
  rclcpp::Node::SharedPtr client_node_;

  // The robot namespace from robot_namespace_editor_;
  std::string robot_namespace_;

  // Robot info service call
  RobotInfo::Request::SharedPtr robot_info_req;

  // Mapping between group names and their robot_info responses
  std::unordered_map<std::string, RobotInfo::Response::SharedPtr> map_group_to_info;

  // Vector containing the joints in the robot
  std::vector<std::string> robot_joints;

  // Vector containing the groups in the robot
  std::vector<std::string> robot_groups;

  // QStringList containing the joints in the robot
  QStringList qrobot_joints;

  // QStringList containing the groups in the robot
  QStringList qrobot_groups;

  // Robot info service client
  rclcpp::Client<RobotInfo>::SharedPtr srv_robot_info;

  QStringList potential_ns_list;
  QCompleter * ns_completer;

  // enables or disables all input fields depending on the given bool
  void enable_elements(const bool enable);

  void update_dropdowns();

  // whether or not the panel has been loaded fully
  bool loaded = false;


  /** -------- Torque Enable Tab -------- */

  // The torque_enable service client
  rclcpp::Client<TorqueEnable>::SharedPtr srv_torque_enable;

  // The torque enable service call
  TorqueEnable::Request::SharedPtr torque_enable_req;


  /** -------- Home/Sleep Tab ------------ */

  // The home/sleep publisher
  rclcpp::Publisher<JointGroupCommand>::SharedPtr pub_joint_group_cmd;

  // The home/sleep jointgroupcommand message
  JointGroupCommand joint_group_cmd;

  /** -------- Reboot Tab ---------------- */

  // The reboot_motors service client
  rclcpp::Client<Reboot>::SharedPtr srv_reboot_motors;

  // The reboot motors service call
  Reboot::Request::SharedPtr reboot_req;


  /** -------- Operating Modes Tab ------- */

  // The operating_modes service client
  rclcpp::Client<OperatingModes>::SharedPtr srv_operating_modes;

  // The operating modes service call
  OperatingModes::Request::SharedPtr opmodes_req;


  /** -------- Get Register Values Tab --- */

  // The get_motor_registers service client
  rclcpp::Client<RegisterValues>::SharedPtr srv_get_motor_registers;

  // The get_motor_registers service call
  RegisterValues::Request::SharedPtr getreg_req;


  /** -------- Emergency Stop Tab -------- */

  // sends the emergency button service call
  void send_system_call();
};  // InterbotixControlPanel

}  // namespace interbotix_xs_rviz

#endif  // INTERBOTIX_XS_RVIZ__INTERBOTIX_CONTROL_PANEL_HPP_
