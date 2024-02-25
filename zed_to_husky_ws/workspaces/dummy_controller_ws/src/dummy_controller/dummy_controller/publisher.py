# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):

    def __init__(self, topic_name: str, node_name="cmd_vel_publisher"):
        super().__init__(node_name)
        
        self.cmd_vel_speed = 0.5 # m/s
        
        self.publisher_ = self.create_publisher(Twist, topic_name, 10)
        self.status = "stop"

    def publish(self):
        msg = Twist()
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0
        msg.angular.x = msg.angular.y = msg.angular.z = 0.0
        
        if self.status == "forward":
            msg.linear.x = self.cmd_vel_speed
        elif self.status == "right":
            msg.angular.z = self.cmd_vel_speed
        elif self.status == "left":
            msg.angular.z = -self.cmd_vel_speed
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % self.status)
