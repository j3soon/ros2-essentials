# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from geometry_msgs.msg import TwistStamped, PoseStamped

# TF
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

# Other
import math
from .pid_controller import PIDController


class URPoseTracking(Node):
    def __init__(self):
        super().__init__("ur_pose_tracking_node")

        # Parameters
        self.declare_parameter("input_pose_topic", "/target_pose")
        self.declare_parameter("output_servo_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("base_frame_id", "world")
        self.declare_parameter("ee_frame_id", "rg2_base_link")
        self.declare_parameter("min_distance_threshold", 0.01)  # meters
        self.declare_parameter("min_angle_threshold", 0.02)  # radians

        # PID parameters
        self.declare_parameter("pid_x_kp", 10.0)
        self.declare_parameter("pid_x_ki", 0.0)
        self.declare_parameter("pid_x_kd", 0.5)
        self.declare_parameter("pid_x_max_speed", 0.5)
        self.declare_parameter("pid_y_kp", 10.0)
        self.declare_parameter("pid_y_ki", 0.0)
        self.declare_parameter("pid_y_kd", 0.5)
        self.declare_parameter("pid_y_max_speed", 0.5)
        self.declare_parameter("pid_z_kp", 10.0)
        self.declare_parameter("pid_z_ki", 0.0)
        self.declare_parameter("pid_z_kd", 0.5)
        self.declare_parameter("pid_z_max_speed", 0.5)
        self.declare_parameter("pid_rot_kp", 5.0)
        self.declare_parameter("pid_rot_ki", 0.0)
        self.declare_parameter("pid_rot_kd", 0.5)
        self.declare_parameter("pid_rot_max_speed", 1.0)

        # Get parameter
        self.publish_frequency = self.get_parameter("publish_frequency").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.ee_frame_id = self.get_parameter("ee_frame_id").value
        self.min_distance_threshold = self.get_parameter("min_distance_threshold").value
        self.min_angle_threshold = self.get_parameter("min_angle_threshold").value

        # Subscribers
        self.target_pose_sub = self.create_subscription(PoseStamped, self.get_parameter("input_pose_topic").value, self.target_pose_callback, 10)

        # Publishers
        self.servo_pub = self.create_publisher(TwistStamped, self.get_parameter("output_servo_topic").value, 10)

        # Timers
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_servo_command_callback)

        # tf
        # NOTE: Use MultiThreadedExecutor in main.py rather than spin_thread=True in TransformListener.
        #       Although the TransformListener will create a Thread and run a SingleThreadedExecutor if spin_thread=True,
        #       this method is not suitable for the current architecture, as we include the TransformListener in this node.
        #       DO NOT CHANGE THIS CONFIGURATION. OTHERWISE, THE TF WILL NOT WORK PROPERLY.
        # Reference:
        # - https://github.com/ros2/geometry2/blob/humble/tf2_ros_py/tf2_ros/transform_listener.py
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # PID Controllers
        self.pid_x = PIDController(self.get_parameter("pid_x_kp").value, self.get_parameter("pid_x_ki").value, self.get_parameter("pid_x_kd").value, self.get_parameter("pid_x_max_speed").value, deadband=self.min_distance_threshold)
        self.pid_y = PIDController(self.get_parameter("pid_y_kp").value, self.get_parameter("pid_y_ki").value, self.get_parameter("pid_y_kd").value, self.get_parameter("pid_y_max_speed").value, deadband=self.min_distance_threshold)
        self.pid_z = PIDController(self.get_parameter("pid_z_kp").value, self.get_parameter("pid_z_ki").value, self.get_parameter("pid_z_kd").value, self.get_parameter("pid_z_max_speed").value, deadband=self.min_distance_threshold)
        self.pid_roll = PIDController(self.get_parameter("pid_rot_kp").value, self.get_parameter("pid_rot_ki").value, self.get_parameter("pid_rot_kd").value, self.get_parameter("pid_rot_max_speed").value, deadband=self.min_angle_threshold)
        self.pid_pitch = PIDController(self.get_parameter("pid_rot_kp").value, self.get_parameter("pid_rot_ki").value, self.get_parameter("pid_rot_kd").value, self.get_parameter("pid_rot_max_speed").value, deadband=self.min_angle_threshold)
        self.pid_yaw = PIDController(self.get_parameter("pid_rot_kp").value, self.get_parameter("pid_rot_ki").value, self.get_parameter("pid_rot_kd").value, self.get_parameter("pid_rot_max_speed").value, deadband=self.min_angle_threshold)

        # Variables
        self.target_pose = None

    def target_pose_callback(self, msg: PoseStamped):
        self.target_pose = msg

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_servo_command_callback(self):
        # Get current end-effector pose and target pose
        current_ee_tf = self.get_current_ee_pose()
        if self.target_pose is None or current_ee_tf is None:
            self.publish_stop_command()
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_roll.reset()
            self.pid_pitch.reset()
            self.pid_yaw.reset()
            return

        # Get current time
        current_time = self.get_clock().now()
        current_time_nanosec = current_time.nanoseconds

        # Compute quaternion difference
        current_q = [current_ee_tf.transform.rotation.x, current_ee_tf.transform.rotation.y, current_ee_tf.transform.rotation.z, current_ee_tf.transform.rotation.w]
        target_q = [self.target_pose.pose.orientation.x, self.target_pose.pose.orientation.y, self.target_pose.pose.orientation.z, self.target_pose.pose.orientation.w]
        current_q_inv = tf_transformations.quaternion_inverse(current_q)
        q_diff = tf_transformations.quaternion_multiply(target_q, current_q_inv)
        if q_diff[3] < 0:
            q_diff = [-x for x in q_diff]

        # Compute errors
        err_x = self.target_pose.pose.position.x - current_ee_tf.transform.translation.x
        err_y = self.target_pose.pose.position.y - current_ee_tf.transform.translation.y
        err_z = self.target_pose.pose.position.z - current_ee_tf.transform.translation.z
        err_roll, err_pitch, err_yaw = tf_transformations.euler_from_quaternion(q_diff)

        # Update PID controllers
        vx = self.pid_x.update(err_x, current_time_nanosec)
        vy = self.pid_y.update(err_y, current_time_nanosec)
        vz = self.pid_z.update(err_z, current_time_nanosec)
        v_roll = self.pid_roll.update(err_roll, current_time_nanosec)
        v_pitch = self.pid_pitch.update(err_pitch, current_time_nanosec)
        v_yaw = self.pid_yaw.update(err_yaw, current_time_nanosec)

        # Create message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = self.base_frame_id

        # Set velocities
        twist_msg.twist.linear.x = float(vx)
        twist_msg.twist.linear.y = float(vy)
        twist_msg.twist.linear.z = float(vz)
        twist_msg.twist.angular.x = float(v_roll)
        twist_msg.twist.angular.y = float(v_pitch)
        twist_msg.twist.angular.z = float(v_yaw)

        # Publish the message
        self.servo_pub.publish(twist_msg)

    def publish_stop_command(self):
        # Create message
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = self.base_frame_id
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

    def get_current_ee_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(self.base_frame_id, self.ee_frame_id, now, timeout=rclpy.duration.Duration(seconds=0.5))
            return transform
        except Exception as e:
            self.get_logger().warn(f"Could not look up transform: {e}")
            return None
