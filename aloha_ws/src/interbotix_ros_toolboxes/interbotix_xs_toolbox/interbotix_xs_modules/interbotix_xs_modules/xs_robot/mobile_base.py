# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Contains an abstract class used to control a mobile base for Interbotix X-Series LoCoBots.

These classes should be used to build out mobile bases for Interbotix X-Series LoCoBots.
"""

from abc import ABC, abstractmethod
from typing import List

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from interbotix_common_modules.common_robot import InterbotixRobotNode
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class InterbotixMobileBaseInterface(ABC):

    def __init__(
        self,
        core: InterbotixRobotNode,
        robot_name: str,
        topic_base_joint_states: str,
        topic_cmd_vel: str = 'cmd_vel',
        nav_timeout_sec: float = 300.0,
        use_nav: bool = False,
    ):
        """
        Construct the InterbotixMobileBaseInterface object.

        :param core: reference to the Node class containing the internal ROS plumbing that drives
            the Python API
        :param robot_name: namespace of the base node (a.k.a the name of the Interbotix base)
        :param topic_base_joint_states: name of the joints states topic that contains the states of
            the base. defaults to `'mobile_base/joint_states'`
        :param topic_cmd_vel: name of the twist topic to which velocity commands should be
            published. defaults to `'cmd_vel'`
        :param nav_timeout_sec: length of time in seconds after which to cancel a navigation goal.
            defaults to `300` (five minutes)
        :param use_nav: whether or not to enable navigation features. requires that nav2 be
            launched. defaults to `False`
        """
        self.core = core
        self.robot_name = robot_name
        self.nav_timeout_sec = nav_timeout_sec
        self.use_nav = use_nav
        self.odom = Odometry()
        self.base_states = JointState()

        cb_group_mobile_base = ReentrantCallbackGroup()

        self.pub_base_twist = self.core.create_publisher(
            msg_type=Twist,
            topic=topic_cmd_vel,
            qos_profile=1,
            callback_group=cb_group_mobile_base,
        )
        self.sub_base_states = self.core.create_subscription(
            msg_type=JointState,
            topic=topic_base_joint_states,
            callback=self._base_states_cb,
            qos_profile=1,
            callback_group=cb_group_mobile_base,
        )
        self.sub_base_odom = self.core.create_subscription(
            msg_type=Odometry,
            topic='/mobile_base/odom',
            callback=self._base_odom_cb,
            qos_profile=1,
            callback_group=cb_group_mobile_base,
        )
        self.client_base_nav_to_pose = ActionClient(
            node=self.core,
            action_type=NavigateToPose,
            action_name='navigate_to_pose',
            callback_group=cb_group_mobile_base,
        )

        self.core.get_logger().info('Initialized InterbotixMobileBaseInterface!')

    def command_velocity_xyaw(
        self,
        x: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        """
        Command a single twist (velocity) by its components to move the robot.

        :param x: (optional) desired speed [m/s] in the 'x' direction (forward/backward). defaults
            to 0
        :param yaw: (optional) desired angular speed [rad/s] around the 'z' axis. defaults to 0
        """
        self.command_velocity(
            twist=Twist(
                linear=Vector3(x=float(x)),
                angular=Vector3(z=float(yaw))
            ),
        )

    def command_velocity_xyaw_for_duration(
        self,
        x: float = 0.0,
        yaw: float = 0.0,
        duration: float = 1.0,
    ) -> None:
        """
        Command twists (velocities) by their components to move the robot for a specified duration.

        :param x: (optional) desired speed [m/s] in the 'x' direction (forward/backward). defaults
            to 0
        :param yaw: (optional) desired angular speed [rad/s] around the 'z' axis. defaults to 0
        :param duration: (optional) length of time in seconds to publish velocity for. defaults to
            1.0
        :details: at the end of the duration, publishes an empty Twist message to halt movement
        """
        self.command_velocity_for_duration(
            twist=Twist(
                linear=Vector3(x=x),
                angular=Vector3(z=yaw)
            ),
            duration=duration,
        )

    def command_velocity_for_duration(self, twist: Twist = Twist(), duration: float = 1.0) -> None:
        """
        Command twists (velocities) message to move the robot for a specified duration.

        :param twist: (optional) desired twist. defaults to empty Twist message (all zeros)
        :param duration: (optional) length of time in seconds to publish velocity for. defaults to
            1.0
        :details: at the end of the duration, publishes an empty Twist message to halt movement
        """
        time_start = self.core.get_clock().now().nanoseconds / 1e9
        r = self.core.create_rate(frequency=10.0)
        while (self.core.get_clock().now().nanoseconds / 1e9 < (time_start + duration)):
            self.pub_base_twist.publish(twist)
            r.sleep()
        self.stop()

    def command_velocity(self, twist: Twist = Twist()) -> None:
        """
        Command a single twist (velocity) message to move the robot.

        :param twist: (optional) desired twist. defaults to empty Twist message (all zeros)
        """
        self.pub_base_twist.publish(twist)

    def command_pose(
        self,
        goal_pose: Pose,
        behavior_tree: str = '',
        blocking=False,
        frame_id: str = 'map',
    ) -> bool:
        """
        Move the base to a given pose in a map (Nav Stack must be enabled!).

        :param goal_pose: desired Pose w.r.t. the map frame that the robot should achieve
        :param behavior_tree: string containing the behavior tree that the NavigateToPose goal
            should specify. defaults to an empty string `''`
        :param blocking: whether the function should wait until the base reaches its goal pose
            before returning control to the user
        :param frame_id: frame name as a string to navigate relative to. defaults to `'map'`
        :return: `True` if the robot successfully reached its goal pose; `False` otherwise. (only
            applies if 'blocking' is `True`)
        :details: note that if 'blocking' is `False`, the function will always return `True`
        """
        if not self.use_nav:
            self.core.get_logger().error('`use_nav` set to `False`. Will not execute navigation.')
            return False
        goal = NavigateToPose.Goal(
            pose=self._stamp_pose(pose=goal_pose, frame_id=frame_id),
            behavior_tree=behavior_tree
        )

        future_send_nav_to_pose_goal = self.client_base_nav_to_pose.send_goal_async(
            goal=goal,
            feedback_callback=self._nav_to_pose_feedback_cb,
        )

        self.core.robot_spin_once_until_future_complete(future_send_nav_to_pose_goal)
        self.goal_handle = future_send_nav_to_pose_goal.result()

        if not self.goal_handle.accepted:
            self.core.get_logger().error(
                f'Navigation goal [{goal_pose.position.x}, {goal_pose.position.y}] was rejected.'
            )
            return False
        self.future_nav = self.goal_handle.get_result_async()
        if blocking:
            while not self.is_nav_complete():
                fb = self.get_nav_to_pose_feedback()
                if Duration.from_msg(fb.navigation_time > Duration(seconds=self.nav_timeout_sec)):
                    self.core.get_logger().error(
                        f'Navigation time ({fb.navigation_time}) exceeds timeout '
                        f'({self.nav_timeout_sec}). Cancelling navigation goal.'
                    )
                    future_cancel_nav_to_pose_goal = self.goal_handle.cancel_goal_async()
                    self.core.robot_spin_once_until_future_complete(
                        future=future_cancel_nav_to_pose_goal
                    )
                return False
        return True

    def command_pose_xyyaw(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        behavior_tree: str = '',
        blocking=False,
        frame_id='map',
    ) -> bool:
        """
        Move the base to a given pose in a map (Nav Stack must be enabled!).

        :param x: desired x [m] w.r.t. the map frame that the robot should achieve
        :param y: desired y [y] w.r.t. the map frame that the robot should achieve
        :param yaw: desired yaw [rad] w.r.t. the map frame that the robot should achieve
        :param behavior_tree: string containing the behavior tree that the NavigateToPose goal
            should specify. defaults to an empty string `''`
        :param blocking: whether the function should wait until the base reaches its goal pose
            before returning control to the user
        :param frame_id: frame name as a string to navigate relative to. defaults to `'map'`
        :return: `True` if the robot successfully reached its goal pose; `False` otherwise. (only
            applies if 'blocking' is `True`)
        :details: note that if 'blocking' is `False`, the function will always return `True`
        """
        q = quaternion_from_euler(0, 0, yaw)
        return self.command_pose(
            goal_pose=Pose(
                position=Point(x=x, y=y),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])),
            behavior_tree=behavior_tree,
            blocking=blocking,
            frame_id=frame_id,
        )

    def stop(self):
        """Publish an empty Twist message that will stop the base's movement."""
        self.pub_base_twist.publish(Twist())

    def get_nav_to_pose_feedback(self) -> NavigateToPose.Feedback:
        """
        Get the most recently received nav to pose feedback message.

        :return: The most recently received NavigateToPose.Feedback message.
        """
        return self.nav_to_pose_feedback

    def is_nav_complete(self):
        """
        Check if the navigate is running.

        :return: `True` if the navigation is running; `False` otherwise
        """
        if not self.future_nav:
            return True
        self.core.robot_spin_once_until_future_complete(future=self.future_nav, timeout_sec=0.1)
        if self.future_nav.result():
            self.nav_status = self.future_nav.result().status
            if self.nav_status != GoalStatus.STATUS_SUCCEEDED:
                self.core.get_logger().error(f"Navigation failed with status '{self.nav_status}'.")
                return True
        else:
            return False

    def get_base_states(self) -> JointState:
        """
        Get the most recently received base JointState message.

        :return: the most recently received base JointState message
        """
        return self.base_states

    def get_odom_xytheta(self) -> List[float]:
        """
        Get the 2D pose of the robot w.r.t. the robot 'odom' frame in [x, y, theta].

        :return: list containing the [x, y, theta] of the robot w.r.t. the odom frame
        """
        return [
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            euler_from_quaternion((
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w
            ))[2]
        ]

    def get_linear_velocity(self) -> Vector3:
        """Return Vector3 of linear velocity."""
        return self.odom.twist.twist.linear

    def get_angular_velocity(self) -> Vector3:
        """Return Vector3 of angular velocity."""
        return self.odom.twist.twist.angular

    def _base_states_cb(self, msg: JointState) -> None:
        """
        Update the base joint states.

        :param msg: ROS JointState message
        """
        self.base_states = msg

    def _base_odom_cb(self, msg: Odometry) -> None:
        """
        Update the odometry of the robot.

        :param msg: ROS Odometry message
        """
        self.odom = msg

    def _nav_to_pose_feedback_cb(self, msg: NavigateToPose.Feedback) -> None:
        """
        Update the nav to pose action feedback.

        :param msg: NavigateToPose.Feedback action feedback
        """
        self.nav_to_pose_feedback = msg

    def _stamp_pose(self, pose: Pose, frame_id: str = 'map') -> PoseStamped:
        """
        Stamp a pose message with the frame_id and the current time.

        :param pose: the Pose message to stamp.
        :param frame_id: (optional) the frame to stamp the Pose message with. defaults to `'map'`
        """
        return PoseStamped(
            pose=pose,
            header=Header(
                frame_id=frame_id,
                stamp=self.core.get_clock().now().to_msg(),
            )
        )

    @abstractmethod
    def reset_odom(self, *args, **kwargs) -> None:
        """Reset odometry to zero."""

    @abstractmethod
    def play_sound(self, *args, **kwargs) -> None:
        """Publish a sound or sounds using the base."""
