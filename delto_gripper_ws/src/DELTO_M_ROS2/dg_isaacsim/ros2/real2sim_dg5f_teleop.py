import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

import math

HOST = '192.168.1.3'
PORT = 10015

# format: 2-byte unsigned short for length + 4byte (float32) values
FORMAT = "<H" + "f" * 64
DATA_SIZE = struct.calcsize(FORMAT)

class MultiRobotToIsaacBridge(Node):
    def __init__(self):
        super().__init__("IsaacSimBridge")

        self.publisher_robot_left = self.create_publisher(JointState, "joint_command_left", 10)
        self.publisher_robot_right = self.create_publisher(JointState, "joint_command_right", 10)

        print("Trying to connect host socket...")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((HOST, PORT))
        print("Connection completed")

        self.joint_state_left = JointState()
        self.joint_state_right = JointState()
        self.joint_state_left.name = [
            "lj_rb_1", "lj_rb_2", "lj_rb_3", "lj_rb_4", "lj_rb_5", "lj_rb_6",
            "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
            "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
            "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
            "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
            "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4"
        ]
        self.joint_state_right.name = [
            "rj_rb_1", "rj_rb_2", "rj_rb_3", "rj_rb_4", "rj_rb_5", "rj_rb_6",
            "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
            "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
            "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
            "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
            "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"
        ]

        self.stop_thread = False
        self.receive_thread = threading.Thread(target=self.receive_robot_state)
        self.receive_thread.start()

        self.publish_timer = self.create_timer(0.01, self.timer_callback_publish_to_isaacsim)

    def receive_robot_state(self):
        while not self.stop_thread:
            try:
                data = self.client_socket.recv(DATA_SIZE)
                if len(data) != DATA_SIZE:
                    print("Incomplete data received, expected:", DATA_SIZE, "but got:", len(data))
                    continue

                unpacked_data = struct.unpack(FORMAT, data)

                # First value: packet size (unused)
                packet_length = unpacked_data[0]
                data = unpacked_data[1:]

                # Data slicing
                l_gripper = data[0:20]
                l_rb_joint = data[20:26]
                l_tcp = data[26:32]
                r_gripper = data[32:52]
                r_rb_joint = data[52:58]
                r_tcp = data[58:64]

                l_gripper_radian = [math.radians(val) for val in data[0:20]]
                l_rb_joint_radian = [math.radians(val) for val in data[20:26]]
                r_gripper_radian = [math.radians(val) for val in data[32:52]]
                r_rb_joint_radian = [math.radians(val) for val in data[52:58]]

                # Update joint messages
                self.joint_state_left.position = l_rb_joint_radian + l_gripper_radian
                self.joint_state_right.position = r_rb_joint_radian + r_gripper_radian
                print("-")
                print("self.joint_state_left: ", self.joint_state_left)
                print("self.joint_state_right: ", self.joint_state_right)

            except Exception as e:
                print("Socket Error:", e)
                self.stop_thread = True

    def timer_callback_publish_to_isaacsim(self):
        self.joint_state_left.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_right.header.stamp = self.get_clock().now().to_msg()

        self.publisher_robot_left.publish(self.joint_state_left)
        self.publisher_robot_right.publish(self.joint_state_right)

    def destroy_node(self):
        self.stop_thread = True
        self.receive_thread.join()
        self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = MultiRobotToIsaacBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
