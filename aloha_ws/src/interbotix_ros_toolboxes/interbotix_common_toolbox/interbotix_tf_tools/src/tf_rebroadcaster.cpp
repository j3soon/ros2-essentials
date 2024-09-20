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

#include "interbotix_tf_tools/tf_rebroadcaster.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace interbotix_tf_tools
{

TFRebroadcaster::TFRebroadcaster(const rclcpp::NodeOptions & options)
: Node("tf_rebroadcaster", options)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("filepath_config");
  this->declare_parameter<std::string>("topic_from");
  this->get_parameter("filepath_config", filepath_config_);
  this->get_parameter("topic_from", topic_from_);

  // Load configuration file
  try {
    // try to load motor_configs yaml file
    config_ = YAML::LoadFile(filepath_config_.c_str());
  } catch (YAML::BadFile & error) {
    // if file is not found or a bad format, shut down
    RCLCPP_FATAL(
      this->get_logger(),
      "Config file at '%s' was not found or has a bad format. Shutting down...",
      filepath_config_.c_str());
    RCLCPP_FATAL(this->get_logger(), "YAML Error: '%s'", error.what());
    return;
  }

  // Get frames config
  YAML::Node all_frames = config_["frames"];
  for (
    YAML::const_iterator frame_itr = all_frames.begin();
    frame_itr != all_frames.end();
    frame_itr++)
  {
    // Get all parent and child frames, insert into map
    std::string frame_parent = frame_itr->first.as<std::string>();
    std::string frame_child = frame_itr->second["child_frame_id"].as<std::string>();
    std::string prefix = frame_itr->second["prefix"].as<std::string>();
    frames_.push_back(Frame{frame_parent, frame_child, prefix});

    // Print out which frames will be rebroadcasted
    if (prefix == "") {
      RCLCPP_INFO(
        this->get_logger(),
        "Will broadcast TF from frame '%s' to frame '%s'.",
        frame_parent.c_str(),
        frame_child.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Will broadcast TF from frame '%s' to frame '%s', prepending prefix '%s'.",
        frame_parent.c_str(),
        frame_child.c_str(),
        prefix.c_str());
    }
  }

  // Create subscription and tf broadcaster
  sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    topic_from_,
    rclcpp::SensorDataQoS().reliable(),
    [this](tf2_msgs::msg::TFMessage msg) {tf_cb(msg);});
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Print out topic info
  RCLCPP_INFO(
    this->get_logger(),
    "Will broadcast TFs from topic '%s' to the 'tf' topic under namespace '%s'.",
    sub_tf_->get_topic_name(),
    this->get_namespace());
}

void TFRebroadcaster::tf_cb(const tf2_msgs::msg::TFMessage & msg)
{
  // TODO(lsinterbotix): optimize
  // Loop over each parent/child frame
  for (auto & frame : frames_) {
    // Loop over every transform in message
    for (const auto & tf_in : msg.transforms) {
      // Check if parent and child match
      if (
        frame.parent_frame_id == tf_in.header.frame_id &&
        frame.child_frame_id == tf_in.child_frame_id)
      {
        // If they do match, rebroadcast to new topic, prepending the prefix if necessary
        geometry_msgs::msg::TransformStamped tf_out;
        tf_out.header.stamp = tf_in.header.stamp;
        tf_out.transform.translation.x = tf_in.transform.translation.x;
        tf_out.transform.translation.y = tf_in.transform.translation.y;
        tf_out.transform.translation.z = tf_in.transform.translation.z;
        tf_out.transform.rotation.x = tf_in.transform.rotation.x;
        tf_out.transform.rotation.y = tf_in.transform.rotation.y;
        tf_out.transform.rotation.z = tf_in.transform.rotation.z;
        tf_out.transform.rotation.w = tf_in.transform.rotation.w;
        if (frame.prefix != "") {
          tf_out.child_frame_id = frame.prefix + tf_in.child_frame_id;
          tf_out.header.frame_id = frame.prefix + tf_in.header.frame_id;
        } else {
          tf_out.child_frame_id = tf_in.child_frame_id;
          tf_out.header.frame_id = tf_in.header.frame_id;
        }
        tf_broadcaster_->sendTransform(tf_out);
        if (!frame.logged) {
          RCLCPP_INFO(
            this->get_logger(),
            "Broadcasted TF from frame '%s' to frame '%s'. This will only log once.",
            tf_out.child_frame_id.c_str(),
            tf_out.header.frame_id.c_str());
          frame.logged = true;
        }
      }
    }
  }
}

}  // namespace interbotix_tf_tools

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(interbotix_tf_tools::TFRebroadcaster)
