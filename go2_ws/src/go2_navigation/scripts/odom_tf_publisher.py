#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# ROS Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

# TF
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__("odom_tf_publisher")

        # Subscriber
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        odom_transform = TransformStamped()
        odom_transform.header.stamp = msg.header.stamp
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_link"
        odom_transform.transform.translation.x = msg.pose.pose.position.x
        odom_transform.transform.translation.y = msg.pose.pose.position.y
        odom_transform.transform.translation.z = msg.pose.pose.position.z
        odom_transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(odom_transform)


def main(args=None):
    rclpy.init(args=args)
    odom_tf_publisher = OdomTFPublisher()
    rclpy.spin(odom_tf_publisher)
    odom_tf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
