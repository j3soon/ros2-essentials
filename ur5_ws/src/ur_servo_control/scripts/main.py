#!/usr/bin/env python3

import rclpy
import subprocess
from ur_servo_control.ur_servo_control import URServoControl


def main(args=None):
    rclpy.init(args=args)

    ur_servo_control = URServoControl()

    try:
        rclpy.spin(ur_servo_control)
    except KeyboardInterrupt:
        # Publish stop command on shutdown
        ur_servo_control.get_logger().info("Shutdown UR servo control node...")
        topic_name = "/forward_velocity_controller/commands"
        msg_type = "std_msgs/msg/Float64MultiArray"
        msg_content = "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
        cmd = ["ros2", "topic", "pub", "-t", "3", topic_name, msg_type, msg_content]
        subprocess.run(cmd)
    finally:
        ur_servo_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
