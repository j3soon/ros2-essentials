// Copyright 2025 Tesollo
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
//    * Neither the name of the Tesollo nor the names of its
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


#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

constexpr double d2r(double deg) { return deg * 3.141592 / 180.0; }

class JointTrajectoryPublisher : public rclcpp::Node {
 public:
  JointTrajectoryPublisher() : Node("joint_trajectory_publisher"), index_(0) {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    timer_ = this->create_wall_timer(
        2s, std::bind(&JointTrajectoryPublisher::timer_callback, this));

    joint_names_ = {
        "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4", "lj_dg_2_1",
        "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4", "lj_dg_3_1", "lj_dg_3_2",
        "lj_dg_3_3", "lj_dg_3_4", "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3",
        "lj_dg_4_4", "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
        "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4", "rj_dg_2_1",
        "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4", "rj_dg_3_1", "rj_dg_3_2",
        "rj_dg_3_3", "rj_dg_3_4", "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3",
        "rj_dg_4_4", "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"};

        angles_ = {
            std::vector<double>(40, 0.0),
        
            {0,        0,        d2r(-40), d2r(-40),
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
        
             0,        0,        d2r(40),  d2r(40),
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0},
        
            {0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80),
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
        
             0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80),
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0},
        
            {0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80),
             0,        0,        0,        0,
             0,        0,        0,        0,
        
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80),
             0,        0,        0,        0,
             0,        0,        0,        0},
        
            {0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80),
             0,        0,        0,        0,
        
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80),
             0,        0,        0,        0},
        
            {0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             d2r(0),   0,        d2r(70),  d2r(70),
        
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        d2r(70),  d2r(70)} 
        
        };
        
  }

 private:
  void timer_callback() {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = angles_[index_];
    point.time_from_start = rclcpp::Duration(1s);

    msg.points.push_back(point);
    RCLCPP_INFO(this->get_logger(), "Publishing trajectory #%zu", index_);
    publisher_->publish(msg);

    index_++;
    if (index_ >= angles_.size()) {
      index_ = 0;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      publisher_;
  std::vector<std::string> joint_names_;
  std::vector<std::vector<double>> angles_;
  size_t index_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}