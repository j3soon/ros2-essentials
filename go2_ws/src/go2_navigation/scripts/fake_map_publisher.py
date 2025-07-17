#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# ROS Messages
from nav_msgs.msg import OccupancyGrid


class FakeMapPublisher(Node):
    def __init__(self):
        super().__init__("fake_map_publisher")

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publisher
        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos_profile)

        # Timer
        self.timer = self.create_timer(1.0, self.publish_map_callback)

        # Map parameters
        self.map_size = 10.0  # meters
        self.resolution = 0.1

        # Create an empty occupancy grid map
        self.width = int(self.map_size / self.resolution)
        self.height = int(self.map_size / self.resolution)
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.origin.position.x = -self.map_size / 2.0
        self.map_msg.info.origin.position.y = -self.map_size / 2.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data = [0] * (self.width * self.height)

    def publish_map_callback(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    fake_map_publisher = FakeMapPublisher()
    rclpy.spin(fake_map_publisher)
    fake_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
