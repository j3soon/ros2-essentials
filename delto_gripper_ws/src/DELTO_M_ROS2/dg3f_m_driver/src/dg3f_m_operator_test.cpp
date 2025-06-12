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
#include <vector>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

constexpr double d2r(double deg) {
    return deg * 3.141592 / 180.0;
}

class TargetJointPublisher : public rclcpp::Node {
public:
    TargetJointPublisher() : Node("joint_trajectory_publisher"), index_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/target_joint", 10);
        timer_ = this->create_wall_timer(2s,
             std::bind(&TargetJointPublisher::timer_callback, this));


        angles_ = {
            {0,        0,       d2r(0), d2r(0),
             0,        0,        0,        0,
             0,        0,        0,        0},

             {0,        0,       d2r(80), d2r(80),
             0,        0,        0,        0,
             0,        0,        0,        0},

            {0,        0,        0,        0,
             0,        0,        d2r(80), d2r(80),
             0,        0,        0,        0},

            {0,        0,        0,        0,
             0,        0,        0,        0,
             0,        0,        d2r(80),  d2r(80)},
        };
    }

private:
    void timer_callback() {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(angles_[index_].size());
        for (size_t i = 0; i < angles_[index_].size(); ++i) {
            msg.data[i] = angles_[index_][i];
        }

        publisher_->publish(msg);

        index_++;

        if (index_ >= angles_.size()) {
            index_ = 0;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std::vector<std::vector<double>> angles_;
    size_t index_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetJointPublisher>());
    rclcpp::shutdown();
    return 0;
}
