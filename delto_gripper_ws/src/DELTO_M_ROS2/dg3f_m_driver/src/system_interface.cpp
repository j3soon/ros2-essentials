// Copyright 2025 TESOLLO
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
//    * Neither the name of the TESOLLO nor the names of its
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

#include "dg3f_driver/system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dg3f_m_driver
{

hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::CallbackReturn::SUCCESS !=
    hardware_interface::SystemInterface::on_init(info))
  {
    return CallbackReturn::ERROR;
  }

  positions_.resize(info_.joints.size(), 0.0);
  velocities_.resize(info_.joints.size(), 0.0);
  efforts_.resize(info_.joints.size(), 0.0);
  effort_commands_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs a command interface.", joint.name.c_str());

      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
      hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name ==
      hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces[2].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs the following state interfaces in this "
        "order: %s, %s",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  delto_ip_ = info.hardware_parameters.at("delto_ip");
  delto_port_ = std::stoi(info.hardware_parameters.at("delto_port"));

  model_ = 0x3F02;
  // std::cout << "Model: " << std::hex << model_ << std::endl;
  if (info.hardware_parameters.at("fingertip_sensor") == "true") {
    fingertip_sensor_ = true;
  } else {
    fingertip_sensor_ = false;
  }

  if (info.hardware_parameters.at("IO") == "true") {
    io_ = true;
  } else {
    io_ = false;
  }

  delto_client_ = std::make_unique<DeltoTCP::Communication>(
    delto_ip_, delto_port_, model_, fingertip_sensor_, io_);
  m_init_thread_ = std::thread(&SystemInterface::init, this);
  m_init_thread_.detach();

  return CallbackReturn::SUCCESS;
}

hardware_interface::SystemInterface::CallbackReturn
SystemInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  if (delto_client_) {
    delto_client_->Disconnect();
  }

  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Deactivated driver");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(
    info_.joints.size() *
    2);                         // position, velocity, effort

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocities_[i]));

    std::cout << "export_state_interfaces: " << info_.joints[i].name << std::endl;
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &effort_commands_[i]));
  }

  return command_interfaces;
}

SystemInterface::return_type SystemInterface::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  return return_type::OK;
}

SystemInterface::CallbackReturn SystemInterface::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Started driver");

  return CallbackReturn::SUCCESS;
}

hardware_interface::SystemInterface::CallbackReturn
SystemInterface::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Stopped driver");
  return CallbackReturn::SUCCESS;
}

void SystemInterface::init() {delto_client_->Connect();}

SystemInterface::return_type SystemInterface::read(
  [[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  try {
    if (!delto_client_) {
      std::cerr << "Client is not initialized" << std::endl;
      return return_type::ERROR;
    }

    DeltoReceivedData received_data;
    try {
      received_data = delto_client_->GetData();
    } catch (const std::exception & e) {
      std::cerr << "Failed to read data: " << e.what() << std::endl;
      return return_type::ERROR;
    }

    // 데이터 크기 검증
    if (received_data.joint.size() != positions_.size()) {
      std::cerr << "Insufficient position data. Expected: " << positions_.size()
                << ", Got: " << received_data.joint.size() << std::endl;
      return return_type::ERROR;
    }

    if (received_data.velocity.size() != velocities_.size()) {
      std::cerr << "Insufficient velocity data. Expected: "
                << velocities_.size()
                << ", Got: " << received_data.velocity.size() << std::endl;
      return return_type::ERROR;
    }

    if (received_data.current.size() != efforts_.size()) {
      std::cerr << "Insufficient current data. Expected: " << efforts_.size()
                << ", Got: " << received_data.current.size() << std::endl;
      return return_type::ERROR;
    }

    positions_ = received_data.joint;
    velocities_ = received_data.velocity;
    efforts_ = received_data.current;
    for (size_t i = 0; i < received_data.joint.size(); ++i) {
      positions_[i] = received_data.joint[i];

      // if (i % 4 == 2) {
      //   positions_[i] = positions_[i] + M_PI / 2;
      // }
    }

    for (size_t i = 0; i < received_data.velocity.size(); ++i) {
      velocities_[i] = received_data.velocity[i];
    }
    for (size_t i = 0; i < received_data.current.size(); ++i) {
      efforts_[i] = received_data.current[i];
    }
    // if (received_data.temperature.size() != temperature_.size()) {

    // tempertaure_ = received_data.temperature;

    // if (received_data.temperature.size() != temperature_.size()) {
    //   std::cerr << "Insufficient temperature data. Expected: "
    //             << temperature_.size()
    //             << ", Got: " << received_data.temperature.size() <<
    //             std::endl;
    //   return return_type::ERROR;
    // }

    return return_type::OK;
  } catch (const std::exception & e) {
    std::cerr << "Unexpected error in read: " << e.what() << std::endl;
    return return_type::ERROR;
  }
}

SystemInterface::return_type SystemInterface::write(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
  std::vector<int> int_duty(effort_commands_.size());

  for (size_t i = 0; i < effort_commands_.size(); ++i) {
    if (effort_commands_[i] > 30.0) {
      int_duty[i] = 300;
      continue;
    } else if (effort_commands_[i] < -30.0) {
      int_duty[i] = -300;
      continue;
    }
    int_duty[i] = static_cast<int>(effort_commands_[i] * 10);
  }
  for (auto & i : int_duty) {
    i *= -1;
  }
  delto_client_->SendDuty(int_duty);

  return return_type::OK;
}
}  // namespace dg3f_m_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dg3f_m_driver::SystemInterface,
  hardware_interface::SystemInterface)
