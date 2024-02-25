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
from dummy_controller.subscriber import ImageSubscriber
from dummy_controller.publisher import CmdVelPublisher


def UpdateCmdVel(left_image_subscriber, right_image_subscriber, cmd_vel_publisher):
    threshold = 10
    
    left_brightness = left_image_subscriber.get_brightness()
    right_brightness = right_image_subscriber.get_brightness()
    
    if left_brightness < threshold and right_brightness < threshold:
        cmd_vel_publisher.status = "stop"
    elif left_brightness > right_brightness:
        cmd_vel_publisher.status = "left"
    elif left_brightness < right_brightness:
        cmd_vel_publisher.status = "right"
    else:
        cmd_vel_publisher.status = "forward"
    
    cmd_vel_publisher.publish()


def main(args=None):
    rclpy.init(args=args)
    
    left_image_subscriber = ImageSubscriber(topic_name="/zed/zed_node/left/image_rect_color", node_name="left_image_subscriber")
    right_image_subscriber = ImageSubscriber(topic_name="/zed/zed_node/right/image_rect_color", node_name="right_image_subscriber")
    cmd_vel_publisher = CmdVelPublisher(topic_name="/a200_0000/cmd_vel")
    
    try:
        while rclpy.ok():
            # As we haven't set a timeout for spin_once, it will block execution until a message is received.
            # Hence, there's no necessity to utilize a sleep function to control the loop rate.
            rclpy.spin_once(left_image_subscriber)
            rclpy.spin_once(right_image_subscriber)
            
            # Update the cmd_vel based on the brightness of the left and right images
            UpdateCmdVel(left_image_subscriber, right_image_subscriber, cmd_vel_publisher)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        left_image_subscriber.destroy_node()
        right_image_subscriber.destroy_node()
        cmd_vel_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
