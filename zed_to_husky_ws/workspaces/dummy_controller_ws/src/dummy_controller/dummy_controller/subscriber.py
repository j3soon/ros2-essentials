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
        self.image_brightness = 0

    def listener_callback(self, msg):
        self.image_brightness = sum(msg.data) / len(msg.data)
        # self.get_logger().info("Received an image, brightness: %d" % self.image_brightness)
    
    def get_brightness(self):
        return self.image_brightness
