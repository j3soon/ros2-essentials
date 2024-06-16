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
import numpy as np
from rclpy.node import Node

# Import laser scan
from sensor_msgs.msg import LaserScan


class LaserSubscriber(Node):

    def __init__(self, node_name="laser_subscriber", degree_range=[-5, 5]):
        super().__init__(node_name)

        self.subscription = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )
        self.subscription

        # Message to store the laser scan.
        self.msg = LaserScan

        # Degree range to calculate the forward view of average range.
        self.degree_range = degree_range

        # Index of the degree range in the message.
        self.degree_range_index = None

    def _get_angle_index(self, angle, angle_increment, angle_min) -> int:

        if angle_increment == 0:
            return 0
        return int((angle - angle_min) / angle_increment)

    def _get_range_index(self, angle_min: float, angle_increment: float) -> list:
        
        # Get the index of the degree range, remember to convert the degree to angle.
        # Maybe we could consider to directly use the angle as input instead of the degree,
        # but it maybe not straightforward to use the angle for the user.
        start = self._get_angle_index(np.deg2rad(self.degree_range[0]), angle_increment, angle_min)
        end = self._get_angle_index(np.deg2rad(self.degree_range[1]), angle_increment, angle_min)

        return list(range(start, end + 1))

    def get_average_range(self) -> float:

        if self.degree_range_index is None:
            self.degree_range_index = self._get_range_index(self.msg.angle_min, self.msg.angle_increment)

        # Get the range in the degree range and remove inf values.
        ranges = self.msg.ranges
        ranges = [ranges[i] for i in self.degree_range_index if ranges[i] != float('inf')]

        if len(ranges) == 0:
            return float('inf')

        return np.mean(ranges)

    def scan_callback(self, msg):

        self.msg = msg
