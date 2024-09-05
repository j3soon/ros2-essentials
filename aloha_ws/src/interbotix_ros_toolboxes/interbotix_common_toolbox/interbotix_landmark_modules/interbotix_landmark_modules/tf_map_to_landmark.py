#!/usr/bin/env python3

# Copyright 2023 Trossen Robotics
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

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node


class PublishMapToLandmarkStaticTF(Node):
    """Publish a transformation between a two fixed frames to separate landmarks from the map."""

    def __init__(self):
        super().__init__('tf_map_to_landmark')
        # publisher to static_transforms
        pub = self.create_publisher(
            TransformStamped,
            'static_transforms',
            qos_profile=1
        )

        original_frame = self.get_parameter_or('~original_frame', 'map')
        fixed_frame = self.get_parameter_or('~fixed_frame', 'landmarks')

        # build identity TF between /orig and /fixed
        tf = TransformStamped()
        tf.header.frame_id = original_frame
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.child_frame_id = fixed_frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        # publish tf and log
        pub.publish(tf)
        self.get_logger().info((
            f"Broadcasted static transformation between '{original_frame}' and '{fixed_frame}'."))

        self.get_logger().info('Shutting down...')


def main():
    rclpy.init()
    PublishMapToLandmarkStaticTF()


if __name__ == '__main__':
    main()
