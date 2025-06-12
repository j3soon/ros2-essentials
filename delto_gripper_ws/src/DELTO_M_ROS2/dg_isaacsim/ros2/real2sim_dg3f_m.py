import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import threading

HOST = '192.168.1.15'   # check your IP with Port
PORT = 10005
FORMAT = "<B" + "h" * 12
DATA_SIZE = struct.calcsize(FORMAT)

class Real2SimBridge(Node):
    def __init__(self):
        super().__init__("Real2SimBridge")

        self.publisher_dg = self.create_publisher(JointState, "dg_joint_command", 10)

        print("Trying to connect host socket...")
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((HOST, PORT))
        print("Connection completed")

        self.joint_state = JointState()
        self.joint_state.name = [
            "j_dg_1_1", "j_dg_1_2", "j_dg_1_3", "j_dg_1_4",
            "j_dg_2_1", "j_dg_2_2", "j_dg_2_3", "j_dg_2_4",
            "j_dg_3_1", "j_dg_3_2", "j_dg_3_3", "j_dg_3_4",
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
                delto_gripper_joints = [math.radians(val / 10.0) for val in unpacked_data[1:21]]
                self.joint_state.position = delto_gripper_joints

                # print("----------------------------------------------------")
                # print("Received Data:")
                # print(f"Delto Gripper Joints: {delto_gripper_joints}")
                # print("----------------------------------------------------")

            except Exception as e:
                print("Socket Error:", e)
                self.stop_thread = True

    def timer_callback_publish_to_isaacsim(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_dg.publish(self.joint_state)

    def destroy_node(self):
        self.stop_thread = True
        self.receive_thread.join()
        self.client_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    real2sim_bridge = Real2SimBridge()
    rclpy.spin(real2sim_bridge)
    real2sim_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
