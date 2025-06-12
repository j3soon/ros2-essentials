#!/usr/bin/env python3

# Copyright 2025 tesollo
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
#    * Neither the name of the tesollo nor the names of its
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


import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration


def d2r(deg):
    return deg * 3.141592 / 180.0


class TargetJointPublisher(Node):
    def __init__(self):
        super().__init__('target_joint_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/target_joint', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.angles = [[0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        ],

                       [d2r(0), d2r(0), d2r(80), d2r(80),
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        ],

                       [0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, d2r(80), d2r(80),
                        0.0, 0.0, 0.0, 0.0,
                        ],

                       [0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, d2r(80), d2r(80),
                        ]]

        self.index = 0

    def timer_callback(self):
        message = Float64MultiArray()
        message.data = self.angles[self.index]
        # message.layout.dim.append(
        #     Float64MultiArray.Dim())
        # message.layout.dim[0].label = 'joint'
        # message.layout.dim[0].size = len(self.angles[self.index])

        # message.points.append(point)
        self.get_logger().info("Joint  #{} publish : {}".format(
            self.index, self.angles[self.index]))
        self.publisher_.publish(message)

        self.index += 1
        if self.index >= len(self.angles):
            self.index = 0


def main(args=None):
    rclpy.init(args=args)
    target_joint_publisher = TargetJointPublisher()
    rclpy.spin(target_joint_publisher)
    target_joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
