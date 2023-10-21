#!/usr/bin/env python3

import rclpy
from deeplab.ros_node import DeepLab_Node


def main(args=None):
    rclpy.init(args=args)

    # Create node
    model_node = DeepLab_Node()
    print("DeepLab node is ready!")
    print("Start inference...")

    # Spin the node so the callback function is called.
    rclpy.spin(model_node)

    # Destroy the node explicitly
    model_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
