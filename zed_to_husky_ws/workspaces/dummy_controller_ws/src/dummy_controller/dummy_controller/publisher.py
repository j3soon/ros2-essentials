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
        self.msg = Twist()

    def forward(self, speed):
        self.status = "forward"
        self.msg.linear.y = self.msg.linear.z = 0.0
        self.msg.angular.x = self.msg.angular.y = self.msg.angular.z = 0.0
        self.msg.linear.x = speed

        self.publish()
        self.get_logger().info('Publishing speed: %lf' % speed)

    def update_status(self):
        self.msg.linear.x = self.msg.linear.y = self.msg.linear.z = 0.0
        self.msg.angular.x = self.msg.angular.y = self.msg.angular.z = 0.0
        
        if self.status == "right":
            self.msg.angular.z = self.cmd_vel_speed
        elif self.status == "left":
            self.msg.angular.z = -self.cmd_vel_speed
        
        self.publish()
        self.get_logger().info('Publishing: "%s"' % self.status)
    
    def publish(self):
        self.publisher_.publish(self.msg)
    
