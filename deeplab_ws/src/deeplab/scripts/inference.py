#!/usr/bin/env python3

import rclpy
from deeplab.ros_node import DeepLab_Node


def main(args=None):
    rclpy.init(args=args)

    # Create node
    model_node = DeepLab_Node()
    model_node.get_logger().info("DeepLab node is ready!")
    model_node.get_logger().info("Start inference...")

    # Spin the node so the callback function is called.
    rclpy.spin(model_node)

    # Destroy the node explicitly
    model_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
