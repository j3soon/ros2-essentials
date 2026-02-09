# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from geometry_msgs.msg import TwistStamped


# Reference:
# - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
class URServoControl(Node):
    def __init__(self):
        super().__init__("ur_servo_control_node")

        # Parameters
        self.declare_parameter("output_servo_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("publish_frequency", 50.0)

        # Get parameter
        self.publish_frequency = self.get_parameter("publish_frequency").value

        # Publishers
        self.servo_pub = self.create_publisher(TwistStamped, self.get_parameter("output_servo_topic").value, 10)

        # Timers
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_servo_command_callback)

    def publish_servo_command_callback(self):
        # Create message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        # Choose reference frame (such as `tool0` or `base_link`)
        twist_msg.header.frame_id = "tool0"

        # Set velocities
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0

        # Publish the message
        self.servo_pub.publish(twist_msg)

    def publish_stop_command(self):
        # Create message
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = "base_link"
        stop_msg.header.stamp = self.get_clock().now().to_msg()

        # Set all velocities to zero
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.linear.y = 0.0
        stop_msg.twist.linear.z = 0.0
        stop_msg.twist.angular.x = 0.0
        stop_msg.twist.angular.y = 0.0
        stop_msg.twist.angular.z = 0.0

        # Publish the stop message
        self.servo_pub.publish(stop_msg)
