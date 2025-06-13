#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


class JointCommandConverter(Node):
    def __init__(self):
        super().__init__("joint_command_converter")

        self.trajectory_subscriber = self.create_subscription(JointTrajectoryControllerState, "/joint_trajectory_controller/controller_state", self.trajectory_callback, 10)
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_command", 10)

    def trajectory_callback(self, msg):
        joint_state_msg = JointState()
        joint_state_msg.header = msg.header
        joint_state_msg.name = msg.joint_names
        joint_state_msg.position = msg.output.positions
        joint_state_msg.velocity = msg.output.velocities
        joint_state_msg.effort = msg.output.effort

        self.joint_state_publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    joint_command_converter = JointCommandConverter()
    rclpy.spin(joint_command_converter)
    joint_command_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
