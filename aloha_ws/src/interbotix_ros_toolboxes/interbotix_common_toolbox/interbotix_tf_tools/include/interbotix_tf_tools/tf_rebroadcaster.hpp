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

#ifndef INTERBOTIX_TF_TOOLS__TF_REBROADCASTER_HPP_
#define INTERBOTIX_TF_TOOLS__TF_REBROADCASTER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "interbotix_tf_tools/visibility_control.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace interbotix_tf_tools
{

struct Frame
{
  std::string parent_frame_id;
  std::string child_frame_id;
  std::string prefix;
  bool logged = false;
};

class TFRebroadcaster : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit TFRebroadcaster(const rclcpp::NodeOptions & options);

private:
  void tf_cb(const tf2_msgs::msg::TFMessage & msg);

  // Subscription to the topic to get TFs from
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;

  // Transform broadcaster to the output topic
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Absolute filepath to the configuration file
  std::string filepath_config_;

  // String containing the topic that the rebroadcaster should listen to TFs from
  std::string topic_from_;

  // YAML node containing configuration info
  YAML::Node config_;

  // Vector containing all frames to be re-broadcasted from the configuration
  std::vector<Frame> frames_;
};

}  // namespace interbotix_tf_tools

#endif  // INTERBOTIX_TF_TOOLS__TF_REBROADCASTER_HPP_
