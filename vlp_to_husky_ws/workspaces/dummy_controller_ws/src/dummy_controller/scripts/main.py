#!/usr/bin/env python3

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
from dummy_controller.publisher import CmdVelPublisher
from dummy_controller.subscriber import LaserSubscriber

def UpdateCmdVelByRange(laser_subscriber, cmd_vel_publisher):

    # Get the average range
    ave_range = laser_subscriber.get_average_range()

    # If the average range is less than 2.5 meter, stop it.
    if ave_range < 2.5:
        cmd_vel_publisher.forward(0.0)
    else:
        cmd_vel_publisher.forward(0.3)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CmdVelPublisher("/a200_0000/cmd_vel")
    laser_subscriber = LaserSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(laser_subscriber)
            UpdateCmdVelByRange(laser_subscriber, cmd_vel_publisher)

    except KeyboardInterrupt:
        cmd_vel_publisher.destroy_node()
        laser_subscriber.destroy_node()


if __name__ == '__main__':
    main()
