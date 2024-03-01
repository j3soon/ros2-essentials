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
import struct
from rclpy.node import Node

from sensor_msgs.msg import Image


class ImageSubscriber(Node):

    def __init__(self, topic_name: str, node_name: str):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            msg_type=Image,
            topic=topic_name,
            callback=self.listener_callback,
            qos_profile=10)
        self.subscription  # prevent unused variable warning
        self.msg = None

    def listener_callback(self, msg):
        self.msg = msg
        # self.get_logger().info("Brightness: %lf" % self.get_brightness())
        # self.get_logger().info("Depth: %lf" % self.get_depth())
    
    def get_brightness(self):
        if self.msg == None:
            return float("inf")
        
        image_brightness = sum(self.msg.data) / len(self.msg.data)
        return image_brightness
    
    def get_depth(self):
        if self.msg == None:
            return float("inf")
        
        # As the original data is an array of char, we will need to convert it into a list of floats.
        # Reference: https://www.stereolabs.com/docs/ros2/depth-sensing
        depth_data = struct.unpack('%sf' % (self.msg.height * self.msg.width), self.msg.data)
        
        center_depth = depth_data[int((self.msg.height / 2) * self.msg.width + (self.msg.width / 2))]
        return center_depth