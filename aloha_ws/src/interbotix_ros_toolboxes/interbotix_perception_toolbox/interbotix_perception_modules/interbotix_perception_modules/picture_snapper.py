#!/usr/bin/env python3

# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import threading

import cv2
from cv_bridge import CvBridge, CvBridgeError
from interbotix_perception_msgs.srv import SnapPicture
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


"""
To start this application, open a terminal on the robot and type:

    ros2 launch interbotix_perception_modules picture_snapper.launch.py

To save a picture, open a new terminal in this directory and type:

    ros2 service call /<apriltag_ns>/snap_picture "filename: '[filename].jpg'"
"""


class PictureSnapper(Node):
    """
    Class to get the latest picture from the camera and save it to a file with a specified name.

    :details: meant to run on the same computer with the AprilTag Single Image Server node; this
        way, all images will be saved to the same computer so that the AprilTag Single Image Server
        node can access them
    """

    def __init__(self):
        """Construct the PictureSnapper node."""
        super().__init__('picture_snapper')
        self.image: Image = None
        self.img_mutex = threading.Lock()

        self.declare_parameter(
            'camera_color_topic',
            '/camera/color/image_raw'
        )
        self.declare_parameter(
            'apriltag_ns',
            'apriltag'
        )
        apriltag_ns = self.get_parameter(
            'apriltag_ns').get_parameter_value().string_value

        self.declare_parameter(
            f'/{apriltag_ns}/picture_snapper/image_save_dir',
            '/tmp/'
        )

        self.camera_color_topic = self.get_parameter(
            'camera_color_topic').get_parameter_value().string_value.strip('/')
        dir_param = self.get_parameter(
            f'/{apriltag_ns}/picture_snapper/image_save_dir').get_parameter_value().string_value

        self.sub_camera_color = self.create_subscription(
            Image,
            f'/{self.camera_color_topic}',
            self.camera_color_cb,
            1
        )
        # directory in which to save pictures (leave this as is)
        self.image_save_dir = os.path.dirname(dir_param)
        self.full_image_save_dir = os.path.join(os.getcwd(), self.image_save_dir)

        try:  # check if directory exists, if not make one
            if not os.path.exists(self.image_save_dir):
                os.makedirs(self.image_save_dir)
            self.get_logger().info(f"Saving images to '{self.full_image_save_dir}'.")
        except OSError as e:  # if we fail (permissions, etc.)
            self.get_logger().error(f'Failed to create directory: {e}.')
            sys.exit(1)

        # wait to receive images (means that we are properly subscribed to the topic)
        rclpy.spin_once(self, timeout_sec=3)  # spin once to process any callbacks
        while (self.image is None and rclpy.ok()):
            self.get_logger().warn(
                f"No Image messages received yet on topic '{self.camera_color_topic}'.",
                throttle_duration_sec=5,
            )
            rclpy.spin_once(self, timeout_sec=1)
        self.create_service(SnapPicture, 'snap_picture', self.snap_picture)
        self.get_logger().info(f"Ready to save images from topic '{self.camera_color_topic}'.")
        self.get_logger().info('Picture Snapper is up!')

    def camera_color_cb(self, msg: Image):
        """
        Get the latest color image through a ROS subscriber callback.

        :param msg: ROS Image message
        """
        with self.img_mutex:
            self.image = msg

    def snap_picture(self, req: SnapPicture.Request, res: SnapPicture.Response):
        """
        Save the latest rgb picture with the desired name from a ROS service callback.

        :param req: ROS 'SnapPicture' Service message request
        :param res: ROS 'SnapPicture' Service message response
        """
        res.success = False
        res.filepath = 'NULL'
        bridge = CvBridge()
        with self.img_mutex:
            if self.image is None:
                self.get_logger().error(f"Got no image from topic '{self.camera_color_topic}'.")
                return res
            try:
                cv_image = bridge.imgmsg_to_cv2(self.image, 'bgr8')
                cv2.imwrite(os.path.join(self.image_save_dir, req.filename), cv_image)
                filepath = os.path.join(self.full_image_save_dir, req.filename)
                self.get_logger().info(f"Image saved to '{filepath}'")
                res.success = True
                res.filepath = filepath
            except OSError as e:
                self.get_logger().error(f'Failed to save image: {e}')
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge failed to process image: {e}')
        return res


def main(args=None):
    rclpy.init(args=args)
    try:
        picture_snapper = PictureSnapper()
        rclpy.spin(picture_snapper)
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
