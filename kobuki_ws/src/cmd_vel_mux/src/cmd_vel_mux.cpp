// Copyright (c) 2012 Yujin Robot, Daniel Stonier, Jorge Santos
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Yujin Robot nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <chrono>
#include <cinttypes>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cmd_vel_mux/cmd_vel_mux.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcpputils/split.hpp"
#include "std_msgs/msg/string.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/
const char * const CmdVelMux::VACANT = "empty";

CmdVelMux::CmdVelMux(rclcpp::NodeOptions options)
: rclcpp::Node("cmd_vel_mux", options.allow_undeclared_parameters(
      true).automatically_declare_parameters_from_overrides(true)), allowed_(VACANT)
{
  std::map<std::string, rclcpp::Parameter> parameters;
  // Check if there are loaded parameters from config file besides sim_time_used
  if (!get_parameters("subscribers", parameters) || parameters.size() < 1) {
    RCLCPP_WARN(get_logger(), "No subscribers configured!");
  } else {
    std::map<std::string, ParameterValues> parsed_parameters = parseFromParametersMap(parameters);
    if (parsed_parameters.empty()) {
      // We ran into some kind of error while configuring, quit
      throw std::runtime_error("Invalid parameters");
    }
    if (!parametersAreValid(parsed_parameters)) {
      throw std::runtime_error("Incomplete parameters");
    }
    configureFromParameters(parsed_parameters);
  }

  param_cb_ =
    add_on_set_parameters_callback(
    std::bind(&CmdVelMux::parameterUpdate, this, std::placeholders::_1));

  output_topic_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  RCLCPP_DEBUG(get_logger(), "CmdVelMux : subscribe to output topic 'cmd_vel'");

  active_subscriber_pub_ = this->create_publisher<std_msgs::msg::String>(
    "active", rclcpp::QoS(1).transient_local());    // latched topic

  // Notify the world that right now nobody is publishing on cmd_vel yet
  auto active_msg = std::make_unique<std_msgs::msg::String>();
  active_msg->data = "idle";
  active_subscriber_pub_->publish(std::move(active_msg));

  RCLCPP_DEBUG(get_logger(), "CmdVelMux : successfully initialized");
}

bool CmdVelMux::parametersAreValid(const std::map<std::string, ParameterValues> & parameters) const
{
  std::set<int64_t> used_priorities;

  for (const std::pair<const std::string, ParameterValues> & parameter : parameters) {
    if (parameter.second.topic.empty()) {
      RCLCPP_WARN(get_logger(), "Empty topic for '%s'", parameter.first.c_str());
      return false;
    }
    if (parameter.second.timeout < 0.0) {
      RCLCPP_WARN(get_logger(), "Missing timeout for '%s', ignoring", parameter.first.c_str());
      return false;
    }
    if (parameter.second.priority < 0) {
      RCLCPP_WARN(get_logger(), "Missing priority for '%s', ignoring", parameter.first.c_str());
      return false;
    }
    if (parameter.second.short_desc.empty()) {
      RCLCPP_WARN(get_logger(), "Empty short_desc for '%s', ignoring", parameter.first.c_str());
      return false;
    }

    if (used_priorities.count(parameter.second.priority) != 0) {
      RCLCPP_WARN(get_logger(), "Cannot have duplicate priorities, ignoring");
      return false;
    }
    used_priorities.insert(parameter.second.priority);
  }

  return true;
}

void CmdVelMux::configureFromParameters(const std::map<std::string, ParameterValues> & parameters)
{
  std::map<std::string, std::shared_ptr<CmdVelSub>> new_map;

  for (const std::pair<const std::string, ParameterValues> & parameter : parameters) {
    const std::string & key = parameter.first;
    const ParameterValues & parameter_values = parameter.second;
    // Check if parameter subscriber has all its necessary values
    if (map_.count(key) != 0) {
      // For names already in the subscribers map, retain current
      // object so we don't re-subscribe to the topic
      new_map[key] = map_[key];
    } else {
      new_map[key] = std::make_shared<CmdVelSub>();
    }

    // update existing or new object with the new configuration

    new_map[key]->name_ = key;
    new_map[key]->values_.priority = parameter_values.priority;
    new_map[key]->values_.short_desc = parameter_values.short_desc;

    if (parameter_values.topic != new_map[key]->values_.topic) {
      // Shutdown the topic if the name has changed so it gets recreated
      // on configuration reload. In the case of new subscribers, topic
      // is empty and shutdown has just no effect
      new_map[key]->values_.topic = parameter_values.topic;
      new_map[key]->sub_ = nullptr;
    }

    if (parameter_values.timeout != new_map[key]->values_.timeout) {
      // Change timer period if the timeout changed
      new_map[key]->values_.timeout = parameter_values.timeout;
      new_map[key]->timer_ = nullptr;
    }
  }

  // Take down the deleted subscriber if it was the one being used as source
  if (allowed_ != VACANT && new_map.count(allowed_) == 0) {
    allowed_ = VACANT;
    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }

  map_ = new_map;

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  for (std::pair<const std::string, std::shared_ptr<CmdVelSub>> & m : map_) {
    const std::string & key = m.first;
    const std::shared_ptr<CmdVelSub> & values = m.second;
    if (!values->sub_) {
      values->sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        values->values_.topic, 10,
        [this, key](const geometry_msgs::msg::Twist::SharedPtr msg) {cmdVelCallback(msg, key);});
      RCLCPP_DEBUG(
        get_logger(), "CmdVelMux : subscribed to '%s' on topic '%s'. pr: %" PRId64 ", to: %.2f",
        values->name_.c_str(), values->values_.topic.c_str(),
        values->values_.priority, values->values_.timeout);
    } else {
      RCLCPP_DEBUG(
        get_logger(), "CmdVelMux : no need to re-subscribe to input topic '%s'",
        values->values_.topic.c_str());
    }

    if (!values->timer_) {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      values->timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
            values->values_.timeout)), [this, key]() {timerCallback(key);});
    }
  }

  RCLCPP_INFO(get_logger(), "CmdVelMux : (re)configured");
}

bool CmdVelMux::addInputToParameterMap(
  std::map<std::string, ParameterValues> & parsed_parameters,
  const std::string & input_name,
  const std::string & parameter_name,
  const rclcpp::Parameter & parameter_value)
{
  if (parameter_name == "topic") {
    std::string topic;

    if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      topic.clear();
    } else if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      topic = parameter_value.as_string();
    } else {
      RCLCPP_WARN(get_logger(), "topic must be a string; ignoring");
      return false;
    }

    if (parsed_parameters.count(input_name) == 0) {
      parsed_parameters.emplace(std::make_pair(input_name, ParameterValues()));
    }
    parsed_parameters[input_name].topic = topic;
  } else if (parameter_name == "timeout") {
    double timeout{-1.0};

    if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      timeout = -1.0;
    } else if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      timeout = parameter_value.as_double();
    } else {
      RCLCPP_WARN(get_logger(), "timeout must be a double; ignoring");
      return false;
    }

    if (parsed_parameters.count(input_name) == 0) {
      parsed_parameters.emplace(std::make_pair(input_name, ParameterValues()));
    }
    parsed_parameters[input_name].timeout = timeout;
  } else if (parameter_name == "priority") {
    int64_t priority{-1};

    if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      priority = -1;
    } else if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      priority = parameter_value.as_int();
      if (priority < 0 || priority > std::numeric_limits<uint32_t>::max()) {
        RCLCPP_WARN(get_logger(), "Priority out of range, must be between 0 and MAX_UINT32");
        return false;
      }
    } else {
      RCLCPP_WARN(get_logger(), "priority must be an integer; ignoring");
      return false;
    }

    if (parsed_parameters.count(input_name) == 0) {
      parsed_parameters.emplace(std::make_pair(input_name, ParameterValues()));
    }
    parsed_parameters[input_name].priority = priority;
  } else if (parameter_name == "short_desc") {
    std::string short_desc;
    if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      short_desc.clear();
    } else if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      short_desc = parameter_value.as_string();
    } else {
      RCLCPP_WARN(get_logger(), "short_desc must be a string; ignoring");
      return false;
    }

    if (parsed_parameters.count(input_name) == 0) {
      parsed_parameters.emplace(std::make_pair(input_name, ParameterValues()));
    }
    parsed_parameters[input_name].short_desc = short_desc;
  } else {
    RCLCPP_WARN(get_logger(), "Invalid input variable '%s'; ignored", parameter_name.c_str());
    return false;
  }

  return true;
}

std::map<std::string, ParameterValues> CmdVelMux::parseFromParametersMap(
  const std::map<std::string,
  rclcpp::Parameter> & parameters)
{
  std::map<std::string, ParameterValues> parsed_parameters;
  // Iterate over all parameters and parse their content
  for (const std::pair<const std::string, rclcpp::Parameter> & parameter : parameters) {
    std::vector<std::string> splits = rcpputils::split(parameter.first, '.');
    if (splits.size() != 2) {
      RCLCPP_WARN(
        get_logger(), "Invalid or unknown parameter '%s', ignoring", parameter.first.c_str());
      continue;
    }

    const std::string & input_name = splits[0];
    const std::string & parameter_name = splits[1];
    const rclcpp::Parameter & parameter_value = parameter.second;
    if (!addInputToParameterMap(parsed_parameters, input_name, parameter_name, parameter_value)) {
      parsed_parameters.clear();
      break;
    }
  }

  return parsed_parameters;
}

void CmdVelMux::cmdVelCallback(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg,
  const std::string & key)
{
  // if subscriber was deleted or the one being called right now just ignore
  if (map_.count(key) == 0) {
    return;
  }

  // Reset timer for this source
  map_[key]->timer_->reset();

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((allowed_ == VACANT) ||
    (allowed_ == key) ||
    (map_[key]->values_.priority > map_[allowed_]->values_.priority))
  {
    if (allowed_ != key) {
      allowed_ = key;

      // Notify the world that a new cmd_vel source took the control
      auto active_msg = std::make_unique<std_msgs::msg::String>();
      active_msg->data = map_[key]->name_;
      active_subscriber_pub_->publish(std::move(active_msg));
    }

    output_topic_pub_->publish(*msg);
  }
}

// The per-topic timerCallback is continually reset as cmd_vel messages are
// received.  Thus, if it ever expires, then the topic hasn't published within
// the specified timeout period and it should be removed.
void CmdVelMux::timerCallback(const std::string & key)
{
  if (allowed_ == key) {
    // No cmd_vel messages timeout happened to currently active source, so...
    allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }
}

rcl_interfaces::msg::SetParametersResult CmdVelMux::parameterUpdate(
  const std::vector<rclcpp::Parameter> & update_parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::map<std::string, rclcpp::Parameter> old_parameters;
  if (!get_parameters("subscribers", old_parameters) || old_parameters.size() <= 1) {
    result.successful = false;
    result.reason = "no parameters loaded";
    return result;
  }

  std::map<std::string, ParameterValues> parameters = parseFromParametersMap(old_parameters);

  // And then merge them
  for (const rclcpp::Parameter & parameter : update_parameters) {
    std::vector<std::string> splits = rcpputils::split(parameter.get_name(), '.');
    if (splits.size() != 3) {
      RCLCPP_WARN(
        get_logger(), "Invalid or unknown parameter '%s', ignoring", parameter.get_name().c_str());
      result.successful = false;
      result.reason = "Invalid or unknown parameter";
      break;
    }
    if (splits[0] != "subscribers") {
      RCLCPP_WARN(
        get_logger(), "Unknown parameter prefix '%s', ignoring", parameter.get_name().c_str());
      result.successful = false;
      result.reason = "Unknown parameter prefix";
      break;
    }

    const std::string & input_name = splits[1];
    const std::string & parameter_name = splits[2];

    if (!addInputToParameterMap(parameters, input_name, parameter_name, parameter)) {
      result.successful = false;
      result.reason = "Invalid parameter";
      break;
    }
  }

  if (result.successful) {
    if (parametersAreValid(parameters)) {
      configureFromParameters(parameters);
    } else {
      result.successful = false;
      result.reason = "Incomplete parameters";
    }
  }

  return result;
}

}  // namespace cmd_vel_mux

RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_mux::CmdVelMux)
