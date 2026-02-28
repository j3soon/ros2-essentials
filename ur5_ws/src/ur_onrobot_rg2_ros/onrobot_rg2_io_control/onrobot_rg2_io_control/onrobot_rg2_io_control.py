# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from ur_msgs.msg import ToolDataMsg
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

# ROS Services
from ur_msgs.srv import SetIO


# Reference:
# - https://www.universal-robots.com/media/1226143/rg2-datasheet-v14.pdf
class OnrobotRG2IOControl(Node):
    def __init__(self):
        super().__init__("onrobot_rg2_io_control_node")

        # Parameters
        self.declare_parameter("input_command_topic", "/rg2/command")
        self.declare_parameter("input_tool_data_topic", "/io_and_status_controller/tool_data")
        self.declare_parameter("output_joint_state_topic", "/rg2/joint_state")
        self.declare_parameter("force_mode", "high")  # Use "high" or "low" force mode

        # Publishers
        self.joint_state_publisher = self.create_publisher(JointState, self.get_parameter("output_joint_state_topic").value, 10)

        # Subscriptions
        self.command_subscription = self.create_subscription(Bool, self.get_parameter("input_command_topic").value, self.command_callback, 10)
        self.tool_data_subscription = self.create_subscription(ToolDataMsg, self.get_parameter("input_tool_data_topic").value, self.tool_data_callback, 10)

        # Services
        self.set_io_client = self.create_client(SetIO, "/io_and_status_controller/set_io")
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/io_and_status_controller/set_io service not available, waiting...")

        # Timers
        JOINT_STATE_FREQUENCY = 125.0  # Hz
        self.joint_state_timer = self.create_timer(1.0 / JOINT_STATE_FREQUENCY, self.publish_joint_state)

        # Variables
        self.MAX_GRIPPER_WIDTH = 10.1  # cm
        self.current_gripper_width = self.MAX_GRIPPER_WIDTH  # cm

        # Initialize the RG2 gripper
        self.set_tool_voltage()
        self.set_force_mode(self.get_parameter("force_mode").value)

    def set_tool_voltage(self):
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_TOOL_VOLTAGE
        request.state = float(SetIO.Request.STATE_TOOL_VOLTAGE_24V)  # RG2 gripper uses 24V as default
        self.set_io_client.call_async(request)

    def set_force_mode(self, mode: str):
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = SetIO.Request.PIN_TOOL_DOUT1  # Digital Output 1 for the force mode
        if mode == "high":
            request.state = 0.0  # High force mode
        else:
            request.state = 1.0  # Low force mode
        self.set_io_client.call_async(request)

    def command_callback(self, msg: Bool):
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = SetIO.Request.PIN_TOOL_DOUT0  # Digital Output 0 for the RG2 gripper
        request.state = 0.0 if msg.data else 1.0  # True = open, False = close
        self.set_io_client.call_async(request)

    def tool_data_callback(self, msg: ToolDataMsg):
        # Convert the voltage reading to gripper width.
        # The official linear mapping provided in the manual (shown below) is less precise:
        #   width = (voltage / UR_VOLTAGE_MAX) * 11.0  # units: cm
        # So we collect the data from the real RG2 gripper and fit a polynomial to the data to get a more accurate mapping.
        voltage = msg.analog_input2
        width = -0.8 * (voltage**2) + 6.6 * voltage - 1.6  # cm
        width = max(0.0, min(width, self.MAX_GRIPPER_WIDTH))
        self.current_gripper_width = width

    def _cm_to_rad(self, width: float) -> float:
        # We map the gripper width to the joint angle linearly, this may not be very accurate.
        min_rad = -0.45
        max_rad = 1.0
        width = max(0.0, min(width, self.MAX_GRIPPER_WIDTH))
        return (width / self.MAX_GRIPPER_WIDTH) * (max_rad - min_rad) + min_rad

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["rg2_gripper_joint"]
        joint_state_msg.position = [self._cm_to_rad(self.current_gripper_width)]
        self.joint_state_publisher.publish(joint_state_msg)
