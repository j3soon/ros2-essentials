# Copyright 2022 Trossen Robotics
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

import copy
import math
import time
from typing import Dict, List, Tuple

from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Quaternion, TransformStamped
from interbotix_common_modules import angle_manipulation as ang
from interbotix_rpi_modules.neopixels import InterbotixRpiPixelInterface
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RobotInfo
import numpy as np
import rclpy
from rclpy.duration import Duration
import tf2_ros
from tf_transformations import quaternion_from_euler
from urdf_parser_py.urdf import URDF

raise NotImplementedError('The hexapod module is not yet compatible with ROS2')


"""
Notes

Transform Names
    - self.T_sf: odometry transform from the 'odom' frame to the hexapod's 'base_footprint' frame;
      only the X, Y, and Yaw values are specified; it is updated when the hexapod moves in the
      world
    - self.T_fb: transform from the 'base_footprint' frame to the hexapod's 'base_link' frame; all
      values can be specified; it is updated when the hexapod moves in place
    - self.T_bc: a dictionary that holds all six static transforms from the hexapod's 'base_link'
      frame to every leg's initial "coxa_link" frame; it is never updated after initialization
    - R_cfcm: rotation matrix specifying the transform from a leg's initial (or fixed) 'coxa_link'
      frame to the present (or moving) 'coxa_link' frame; it is updated whenever the hexapod moves
    - p_femur: point specifying the transform from a leg's 'femur_link' frame to its 'foot_link'
      frame; it is updated whenever the hexapod moves
    - p_cm: point specifying the transform from a leg's present (or moving) 'coxa_link' frame to
      its 'foot_link' frame; it is updated whenever the hexapod moves
    - p_cf: point specifying the transform from a leg's initial (or fixed) 'coxa_link' frame to its
      'foot_link' frame; it is updated whenever the hexapod moves
    - p_b: point specifying the transform from the hexapod's 'base_link' frame to a specified leg's
      'foot_link' frame; it is updated whenever the hexapod moves
    - p_f: point specifying the transform from the hexapod's 'base_footprint' frame to a specified
      leg's 'foot_link' frame; it is updated when the hexapod moves in the world

Gaits The code supports the tripod gait and the modified versions of the ripple and wave gaits that
are discussed at https://hexyrobot.wordpress.com/2015/11/20/common-walking-gaits-for-hexapods/. The
implementation of these gaits were all custom developed using two sinusoid wave functions. The
first sin function was used to determine what the x, y, and yaw values for each leg should be while
the second sin function was used to determine what the z value for each leg should be. A 'period'
represents one complete wave cycle of the second sin function signifying the time it takes for a
given leg to start and finish one 'swing' phase. In the tripod gait, a 'period' also represents the
time it takes for a leg to start and finish one 'stance' phase. So, the number of periods needed
for each gait cycle are:
    - Tripod: 2 periods -> one for the 'swing' phase and one for the 'stance' phase
    - Ripple: 3 periods -> one for the 'swing' phase and two for the 'stance' phase
    - Wave: 6 periods -> one for the 'swing' phase and five for the 'stance' phase
This means that the tripod gait is 3 times faster than the wave gait and 1.5 times faster than the
ripple gait. However, the inverse holds true for stability.
"""


class InterbotixHexapodXS:
    """Standalone Module to control an Interbotix Hexapod."""

    def __init__(self, robot_model: str, robot_name: str = None, position_p_gain: int = 800):
        """
        Construct the InterbotixHexapodXS onject.

        :param robot_model: Interbotix Hexapod model (ex. 'mark4')
        :param robot_name: (optional) defaults to value given to 'robot_model'; this can be
            customized to best suit the user's needs
        :param position_p_gain: (optional) passthrough to the Position_P_Gain register on all
            hexapod servos - sets the desired proportional gain
        """
        self.core = InterbotixRobotXSCore(robot_model=robot_model, robot_name=robot_name)
        self.hex = InterbotixHexapodXSInterface(core=self.core, position_p_gain=position_p_gain)
        self.pixels = InterbotixRpiPixelInterface(robot_name=self.core.robot_name)


class InterbotixHexapodXSInterface:
    """Definition of the Interbotix Hexapod Module."""

    inc_prev = 0
    """Latest increment during the gait cycle"""

    period_cntr = 0
    """Used to count a period (self.num_steps/2.0) during the wave or ripple gait cycles"""

    num_steps = 20.0
    """Number of steps in one wave of the first sinusoid cycle"""

    step_cntr = 1
    """Counts the number of steps during a single gait cycle"""

    gait_types = ['tripod', 'ripple', 'wave']
    """Three supported gaits that can be selected"""

    gait_factors = {'tripod': 2.0, 'ripple': 3.0, 'wave': 6.0}
    """
    Gait factors that modify the first sinusoid function mentioned above based on the selected gait
    """

    wave_legs = [
        'right_front', 'left_front', 'right_middle',
        'left_middle', 'right_back', 'left_back'
    ]
    """
    Leg 'queue' when doing the wave gait; after every period, the first element is taken out and
    appended to the end of the list
    """

    ripple_legs = {
        'first': ['left_middle', 'right_front'],
        'second': ['left_back', 'right_middle'],
        'third': ['left_front', 'right_back']
    }
    """Dictionary to keep track of which two legs move together during the ripple gait"""

    ripple_leg_pairs = ['first', 'second', 'third']
    """
    Leg pair 'Queue' when doing the ripple gait; after every period, the first element is taken
    out and appended to the end of the list
    """

    leg_list = [
        'left_back', 'left_middle', 'left_front',
        'right_front', 'right_middle', 'right_back'
    ]
    """List of all legs in the hexapod"""

    leg_mode_on = False
    """Boolean dictating whether or no 'individual leg control' is on or not"""

    foot_points: Dict[str, List[float]] = {}
    """Dictionary that contains the current feet positions for each leg"""

    home_foot_points: Dict[str, List[float]] = {}
    """
    Dictionary that contains the 'home' feet positions for each leg before starting a gait cycle
    """

    sleep_foot_points: Dict[str, List[float]] = {}
    """Dictionary that contains the 'sleep' feet positions for each leg"""

    home_height: float = 0.0
    """
    The 'z' component of `self.T_fb` specifying the height of the 'base_link' frame relative to the
    'base_footprint' frame
    """

    sleep_height: float = 0.0
    """
    The 'z' component of self.T_fb specifying the height of the 'base_link' frame relative to the
    'base_footprint' frame when sleeping
    """

    bottom_height: float = 0.0
    """Height difference between the 'base_link' frame and the 'base_bottom_link' frame"""

    T_bc: Dict[str, np.ndarray] = {}
    """
    Dictionary containing the static transforms of all six 'coxa_link' frames relative to the
    'base_link' frame
    """

    coxa_length: float = None
    """Length [meters] of the coxa_link"""

    femur_length: float = None
    """Length [meters] of the femur_link"""

    tibia_length: float = None
    """Length [meters] of the tibia_link"""

    femur_offset_angle: float = None
    """
    Offset angle [rad] that makes the tibia_link frame coincident with a line shooting out of the
    coxa_link frame that's parallel to the ground
    """

    tibia_offset_angle: float = None
    """
    Offset angle [rad] that makes the foot_link frame coincident with a line shooting out of the
    coxa_link frame that's parallel to the ground
    """

    def __init__(self, core: InterbotixRobotXSCore, position_p_gain: int):
        """
        Construct the InterbotixHexapodXSInterface class.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        """
        # Reference to the InterbotixRobotXSCore object
        self.core = core

        # Desired Proportional gain for all servos
        self.position_p_gain = position_p_gain

        # Dictionary to keep track of where each leg's foot is during the wave gait
        self.wave_incs = {leg: 0 for leg in self.wave_legs}

        # Populate leg_time_map with defaults
        self.leg_time_map['all'] = {'move': 0, 'accel': 0}

        # Dictionary to keep track of where each leg pair's feet are during the ripple gait
        self.ripple_incs = {pair: 0 for pair in self.ripple_leg_pairs}

        # Keeps track of the moving & accel times for each joint group
        self.leg_time_map = {leg: {'move': 0, 'accel': 0} for leg in self.leg_list}

        # Odometry transform specifying the 'base_footprint' frame relative to the 'odom' frame
        self.T_sf = np.identity(4)

        # Body transform specifying the 'base_link' frame relative to the 'base_footprint' frame
        self.T_fb = np.identity(4)

        self.get_urdf_info()

        # ROS PoseStamped message to publish self.T_sf to its own topic
        self.pose = PoseStamped()

        # ROS Transform that holds self.T_sf and is published to the /tf topic
        self.t_sf = TransformStamped()

        # ROS Transform that holds self.T_fb and is published to the /tf topic
        self.t_fb = TransformStamped()

        self.br = tf2_ros.TransformBroadcaster()
        self.initialize_transforms()
        self.info: RobotInfo.Response = self.core.srv_get_info('group', 'all')

        # Map joint names to their positions in the upper/lower and sleep position arrays
        self.info_index_map = dict(zip(self.info.joint_names, range(len(self.info.joint_names))))

        # ROS Message to command all 18 joints in the hexapod simultaneously
        self.hexapod_command = JointGroupCommand(name='all', cmd=[0] * self.info.num_joints)
        self.initialize_start_pose()

        # ROS Publisher to publish self.T_sf as a PoseStamped message
        self.pub_pose = self.core.create_publisher(
            PoseStamped,
            f'/{self.core.robot_name}/pose',
            1
        )

        # ROS Timer to publish transforms to the /tf and /odom topics at a fixed rate
        self.tmr_transforms = self.core.create_timer(Duration(seconds=0.04), self.publish_states)
        print('Initialized InterbotixHexapodXSInterface!\n')

    def get_urdf_info(self) -> None:
        """Parse the URDF and populates the appropiate variables with link information."""
        full_rd_name = f'/{self.core.robot_name}/robot_description'
        self.core.declare_parameter(full_rd_name)
        self.core.get_parameter(full_rd_name).get_parameter_value().string_value
        # while not rospy.has_param(full_rd_name):
        #     pass
        robot_description = URDF.from_parameter_server(key=full_rd_name)

        for leg in self.leg_list:
            joint_object = next(
                (joint for joint in robot_description.joints if joint.name == (f'{leg}_coxa')),
                None)
            T_bc = np.identity(4)
            T_bc[:3, 3] = joint_object.origin.xyz
            T_bc[:3, :3] = ang.euler_angles_to_rotation_matrix(joint_object.origin.rpy)
            self.T_bc[leg] = T_bc

        femur_joint = next(
            (joint for joint in robot_description.joints if joint.name == 'left_front_femur')
        )
        self.coxa_length = femur_joint.origin.xyz[0]

        tibia_joint = next(
            (joint for joint in robot_description.joints if joint.name == 'left_front_tibia')
        )
        femur_x = tibia_joint.origin.xyz[0]
        femur_z = tibia_joint.origin.xyz[2]
        self.femur_offset_angle = abs(math.atan2(femur_z, femur_x))
        self.femur_length = math.sqrt(femur_x**2 + femur_z**2)

        foot_joint = next(
            (joint for joint in robot_description.joints if joint.name == 'left_front_foot')
        )
        tibia_x = foot_joint.origin.xyz[0]
        tibia_z = foot_joint.origin.xyz[2]
        self.tibia_offset_angle = abs(math.atan2(tibia_z, tibia_x)) - self.femur_offset_angle
        self.tibia_length = math.sqrt(tibia_x**2 + tibia_z**2)

        bottom_joint = next(
            (joint for joint in robot_description.joints if joint.name == 'base_bottom')
        )
        self.bottom_height = abs(bottom_joint.origin.xyz[2])
        self.home_height = self.bottom_height + 0.05

    def initialize_transforms(self) -> None:
        """Intialize the static components of the ROS transforms."""
        self.pose.header.frame_id = f'{self.core.robot_name}/odom'
        self.pose.pose.orientation.w = 1.0
        self.t_sf.header.frame_id = f'{self.core.robot_name}/odom'
        self.t_sf.child_frame_id = f'{self.core.robot_name}/base_footprint'
        self.t_sf.transform.rotation.w = 1.0
        self.t_fb.header.frame_id = f'{self.core.robot_name}/base_footprint'
        self.t_fb.child_frame_id = f'{self.core.robot_name}/base_link'
        self.t_fb.transform.rotation.w = 1.0

    def initialize_start_pose(self) -> None:
        """Find the initial foot position for each leg relative to the 'base_footprint' frame."""
        self.T_fb[2, 3] = self.bottom_height
        for leg in self.leg_list:
            theta_1 = self.info.joint_sleep_positions[self.info_index_map[leg + '_coxa']]
            theta_2 = self.info.joint_sleep_positions[self.info_index_map[leg + '_femur']]
            theta_3 = self.info.joint_sleep_positions[self.info_index_map[leg + '_tibia']]
            self.sleep_foot_points[leg] = self.solve_fk([theta_1, theta_2, theta_3], leg)
            self.sleep_height = self.bottom_height - self.sleep_foot_points[leg][2]
            self.sleep_foot_points[leg][2] = 0
        self.home_foot_points = copy.deepcopy(self.sleep_foot_points)
        self.foot_points = copy.deepcopy(self.home_foot_points)
        self.core.srv_set_reg('group', 'all', 'Position_P_Gain', self.position_p_gain)
        self.reset_hexapod('home')
        self.move_in_world()

    def solve_fk(self, theta: float, leg: str) -> List[float]:
        """
        Get the specified leg's foot position relative to the 'base_footprint' frame.

        :param theta: list specifying the desired coxa, femur, and tibia joint values
        :param leg: name of the leg to perform forward-kinematics on
        :return: 3-element list specifying the foot point relative to the 'base_footprint' frame
        """
        x = (
            self.femur_length
            * math.cos(theta[1] + self.femur_offset_angle)
            + self.tibia_length
            * math.cos(theta[1] + self.femur_offset_angle + theta[2] + self.tibia_offset_angle)
        )

        z = (
            -self.femur_length
            * math.sin(theta[1] + self.femur_offset_angle)
            - self.tibia_length
            * math.sin(theta[1] + self.femur_offset_angle + theta[2] + self.tibia_offset_angle)
        )

        R_cfcm = np.identity(3)
        R_cfcm[:2, :2] = ang.yaw_to_rotation_matrix(theta[0])

        p_femur = [x, 0, z]
        p_cm = np.add(p_femur, [self.coxa_length, 0, 0])
        p_cf = np.dot(R_cfcm, p_cm)
        p_b = np.dot(self.T_bc[leg], np.r_[p_cf, 1])
        p_f = np.dot(self.T_fb, p_b)

        return [p_f[0], p_f[1], p_f[2]]

    def solve_ik(
        self,
        p_f: List[float],
        leg: str,
        mod_value: float = 0
    ) -> Tuple[List[float], bool]:
        """
        Get the desired joint angles to move a leg's foot to the right position.

        :param p_f: 3-element list specifying the desired leg's foot position relative to the
            'base_footprint' frame
        :param leg: name of the leg to perform inverse-kinematics on
        :param mod_value: (optional) relative distance value by which to tighten or widen the
            hexapod stance [m]
        :return: 3-element list and boolean specifying the required joint angles and if the
            function was successful respectively
        """
        p_b = np.dot(ang.trans_inv(self.T_fb), np.r_[p_f, 1])
        p_cf = np.dot(ang.trans_inv(self.T_bc[leg]), p_b)
        theta_1 = math.atan2(p_cf[1], p_cf[0])

        R_cfcm = np.identity(3)
        R_cfcm[:2, :2] = ang.yaw_to_rotation_matrix(theta_1)

        p_cm = np.dot(R_cfcm.T, p_cf[:3])
        p_cm[0] += mod_value
        p_femur = np.subtract(p_cm, [self.coxa_length, 0, 0])
        try:
            theta_3 = math.acos(
                (p_femur[0]**2 + p_femur[2]**2 - self.femur_length**2 - self.tibia_length**2)
                / (2 * self.femur_length * self.tibia_length)
            )
            theta_2 = -(
                math.atan2(
                    p_femur[2],
                    p_femur[0])
                + math.atan2(
                    (self.tibia_length * math.sin(theta_3)),
                    (self.femur_length + self.tibia_length * math.cos(theta_3))
                )
            )
            return (
                [theta_1, theta_2 - self.femur_offset_angle, theta_3 - self.tibia_offset_angle],
                True)
        except ValueError:
            return [0, 0, 0], False

    def modify_stance(self, mod_value: float) -> bool:
        """
        Adjust the hexapod's stance to be wider or narrower.

        :param mod_value: relative distance value by which to tighten or widen the hexapod stance
            [m]
        :return: `True` if function completed successfully; `False` otherwise
        """
        new_foot_points = {}
        for leg, point in self.foot_points.items():
            theta_list, success = self.solve_ik(point, leg, mod_value)
            if success:
                new_foot_points[leg] = self.solve_fk(theta_list, leg)
            else:
                return False
        self.foot_points = new_foot_points
        return self.move_in_world()

    def reset_hexapod(self, pose_type: str = 'home') -> None:
        """
        Reset the hexapod to its 'home' or 'sleep' pose.

        :param pose_type: (optional) desired pose
        """
        self.core.get_logger().info(f'Going to {pose_type} pose.')
        self.T_fb = np.identity(4)
        self.T_fb[2, 3] = self.home_height
        self.move_in_place()
        if (self.foot_points != self.home_foot_points):
            self.foot_points = copy.deepcopy(self.home_foot_points)
            self.move_in_world()
        if pose_type == 'sleep':
            if (self.foot_points != self.sleep_foot_points):
                self.foot_points = copy.deepcopy(self.sleep_foot_points)
                self.move_in_world()
            self.T_fb[2, 3] = self.sleep_height
            self.move_in_place()
        self.set_trajectory_time('all', 0.150, 0.075)

    def update_tsf_transform(self, moving_time: float) -> bool:
        """
        Update the `self.T_sf` transform.

        :param moving_time: time [sec] it takes for each motor to move a step
        :details: Message is future dated by 'moving_time' milliseconds since that's the amount of
                  time it takes for the motors to move
        """
        self.t_sf.transform.translation.x = self.T_sf[0, 3]
        self.t_sf.transform.translation.y = self.T_sf[1, 3]
        rpy = ang.rotation_matrix_to_euler_angles(self.T_sf[:3, :3])
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        self.t_sf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        self.t_sf.header.stamp = self.core.get_clock().now() + Duration(seconds=moving_time)
        self.pose.pose.position = Point(self.T_sf[0, 3], self.T_sf[1, 3], 0)
        self.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.pose.header.stamp = (
            self.core.get_clock().now() + Duration(seconds=moving_time)
        ).to_msg()

    def update_tfb_transform(self, moving_time: float):
        """
        Update the `self.T_fb` transform.

        :details: Message is future dated by 'moving_time' since that's the
                  amount of time it takes for the motors to move
        """
        self.t_fb.transform.translation.x = self.T_fb[0, 3]
        self.t_fb.transform.translation.y = self.T_fb[1, 3]
        self.t_fb.transform.translation.z = self.T_fb[2, 3]
        rpy = ang.rotation_matrix_to_euler_angles(self.T_fb[:3, :3])
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        self.t_fb.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        self.t_fb.header.stamp = (
            self.core.get_clock().now() + Duration(seconds=moving_time)
        ).to_msg()

    def update_joint_command(self, point: List[float], leg: str) -> bool:
        """
        Update the ROS message containing the joint commands with new values.

        :param point: 3-element list specifying the desired foot position (relative to the
            'base_footprint' frame) for a given leg
        :param leg: name of the leg to be commanded
        :return: `True` if function completed successfully; `False` otherwise
        """
        theta, success = self.solve_ik(point, leg)
        if not success:
            return False
        theta_names = [f'{leg}_coxa', f'{leg}_femur', f'{leg}_tibia']
        for x in range(len(theta_names)):
            if not (
                (self.info.joint_lower_limits[self.info_index_map[theta_names[x]]] <= theta[x]) and
                (theta[x] <= self.info.joint_upper_limits[self.info_index_map[theta_names[x]]])
            ):
                return False
        self.hexapod_command.cmd[self.info_index_map[f'{leg}_coxa']] = theta[0]
        self.hexapod_command.cmd[self.info_index_map[f'{leg}_femur']] = theta[1]
        self.hexapod_command.cmd[self.info_index_map[f'{leg}_tibia']] = theta[2]
        return True

    def publish_states(self) -> None:
        """
        Continously publish transforms through a ROS Timer callback function.

        :details: if a transform is not being updated, then wait until the time stamp is no longer
            in the future before publishing it with the current ROS time (prevents jumps back in
            time)
        """
        time = self.core.get_clock().now().to_msg()
        if self.t_sf.header.stamp < time:
            self.t_sf.header.stamp = time
            self.pose.header.stamp = time
        if self.t_fb.header.stamp < time:
            self.t_fb.header.stamp = time
        self.br.sendTransform(self.t_sf)
        self.br.sendTransform(self.t_fb)
        self.pub_pose.publish(self.pose)

    def set_trajectory_time(
        self,
        group: str,
        moving_time: float = 1.0,
        accel_time: float = 0.3
    ) -> None:
        """
        Command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers.

        :param group: (optional) name of the leg to control (or 'all' for all legs)
        :param moving_time: (optional) time in seconds that each motor should move
        :param accel_time: time in seconds that each motor should accelerate
        """
        if (group == 'all' and self.leg_mode_on):
            self.core.srv_set_reg(
                'group',
                'all',
                'Profile_Velocity',
                int(moving_time * 1000)
            )
            self.core.srv_set_reg(
                'group',
                'all',
                'Profile_Acceleration',
                int(accel_time * 1000)
            )
            for leg in self.leg_list:
                self.leg_time_map[leg] = {'move': moving_time, 'accel': accel_time}
            self.leg_time_map['all'] = {'move': moving_time, 'accel': accel_time}
            self.leg_mode_on = False
        else:
            times = self.leg_time_map[group]
            if (moving_time != times['move']):
                times['move'] = moving_time
                self.core.srv_set_reg(
                    'group',
                    group,
                    'Profile_Velocity',
                    int(moving_time * 1000)
                )
            if (accel_time != times['accel']):
                times['accel'] = accel_time
                self.core.srv_set_reg(
                    'group',
                    group,
                    'Profile_Acceleration',
                    int(accel_time * 1000)
                )

    def move_leg(
        self,
        leg: str,
        p_f_inc: List[float] = [0, 0, 0],
        moving_time: float = 0.15,
        accel_time: float = 0.075,
        blocking: bool = True
    ) -> bool:
        """
        Move the selected leg's foot position relative to its current foot position.

        :param leg: (optional) name of the leg to move
        :param p_f_inc: (optional) desired relative point to add to the current foot_point
        :param moving_time: (optional) time [sec] that each joint should spend moving
        :param accel_time: (optional) time [sec] that each joint should spend accelerating
        :param blocking: (optional) `True` if the function should wait 'moving_time' seconds before
            returning
        :return: `True` if function completed successfully; `False` otherwise
        """
        self.leg_mode_on = True
        self.set_trajectory_time(leg, moving_time, accel_time)
        point = list(self.foot_points[leg])
        target_point = np.add(point, p_f_inc)
        theta, success = self.solve_ik(target_point, leg)
        if not success:
            return False
        theta_names = [f'{leg}_coxa', f'{leg}_femur', f'{leg}_tibia']
        for x in range(len(theta_names)):
            if not (
                (self.info.joint_lower_limits[self.info_index_map[theta_names[x]]] <= theta[x]) and
                (theta[x] <= self.info.joint_upper_limits[self.info_index_map[theta_names[x]]])
            ):
                return False
        self.core.pub_group.publish(JointGroupCommand(name=leg, cmd=theta))
        self.foot_points[leg] = list(target_point)
        if blocking:
            time.sleep(moving_time)
        return True

    def move_in_place(
        self,
        x: float = None,
        y: float = None,
        z: float = None,
        roll: float = None,
        pitch: float = None,
        yaw: float = None,
        moving_time: float = 1.0,
        accel_time: float = 0.3,
        blocking: bool = True
    ) -> bool:
        """
        Move the hexapod 'base_link' frame in place.

        :param x: (optional) desired 'x' component of self.T_fb
        :param y: (optional) desired 'y' component of self.T_fb
        :param z: (optional) desired 'z' component of self.T_fb
        :param roll: (optional) desired 'roll' component of self.T_fb
        :param pitch: (optional) desired 'pitch' component of self.T_fb
        :param yaw: (optional) desired 'yaw' component of self.T_fb
        :param moving_time: (optional) time [sec] that each joint should spend moving
        :param accel_time: (optional) time [sec] that each joint should spend accelerating
        :param blocking: (optional) `True` if the function should wait 'moving_time' seconds before
            returning; `False` otherwise
        :return: `True` if function completed successfully; `False` otherwise
        """
        self.set_trajectory_time('all', moving_time, accel_time)
        T_fb = np.array(self.T_fb)
        if x is not None:
            self.T_fb[0, 3] = x
        if y is not None:
            self.T_fb[1, 3] = y
        if z is not None:
            self.T_fb[2, 3] = z

        rpy = ang.rotation_matrix_to_euler_angles(self.T_fb[:3, :3])
        if roll is not None:
            rpy[0] = roll
        if pitch is not None:
            rpy[1] = pitch
        if yaw is not None:
            rpy[2] = yaw
        self.T_fb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
        for leg, point in self.foot_points.items():
            if not self.update_joint_command(point, leg):
                self.T_fb = T_fb
                return False
        self.core.pub_group.publish(self.hexapod_command)
        self.update_tfb_transform(moving_time)
        if blocking:
            time.sleep(moving_time)
        return True

    def move_in_world(
        self,
        x_stride: float = 0,
        y_stride: float = 0,
        yaw_stride: float = 0,
        max_foot_height: float = 0.04,
        num_steps: float = 20.0,
        gait_type='tripod',
        mp: float = 0.150,
        ap: float = 0.075,
        num_cycles: int = 1,
        cycle_freq: float = None
    ) -> bool:
        """
        Move the hexapod 'base_footprint' frame relative to the 'odom' frame.

        :param x_stride: (optional) desired positive/negative distance to cover in a gait cycle
            relative to the base_footprint's X-axis
        :param y_stride: (optional) desired positive/negative distance to cover in a gait cycle
            relative to the base_footprint's Y-axis
        :param yaw_stride: (optional) desired positive/negative distance to cover in a gait cycle
            around the base_footprint's Z-axis
        :param max_foot_height: (optional) max height [meters] that a leg's foot will be lifted
            during the 'swing' phase
        :param num_steps: (optional) number of steps to complete one wave in the first sinusoid
            function
        :param gait_type: (optional) desired gait to use
        :param mp: (optional) time [sec] that each joint should spend moving per step
        :param ap: (optional) time [sec] that each joint should spend accelerating per step
        :param num_cycles: (optional) number of gait cycles to complete before exiting
        :param cycle_freq: (optional) frequency at which the gait cycle should run; defaults to
            'num_steps'
        :return: `True` if function completed successfully; False otherwise
        """
        self.set_trajectory_time('all', mp, ap)
        self.num_steps = num_steps
        num_steps_in_cycle = self.num_steps * self.gait_factors[gait_type]/2.0
        if cycle_freq is None:
            cycle_freq = num_steps
        rate = self.core.create_rate(frequency=cycle_freq)
        for _ in range(num_cycles):
            self.step_cntr = 1
            self.inc_prev = 0
            while (self.step_cntr <= num_steps_in_cycle and rclpy.ok()):
                inc = 1/self.gait_factors[gait_type] \
                    * 0.5 * (1 + math.sin(2*np.pi*(self.step_cntr/self.num_steps) - np.pi/2))
                foot_height = max_foot_height * \
                    0.5 * (1 + math.sin(4*np.pi*(self.step_cntr/self.num_steps) - np.pi/2))

                success = False
                if gait_type == 'tripod':
                    success = self.tripod_gait(x_stride, y_stride, yaw_stride, inc, foot_height)
                elif gait_type == 'ripple':
                    success = self.ripple_gait(x_stride, y_stride, yaw_stride, inc, foot_height)
                elif gait_type == 'wave':
                    success = self.wave_gait(x_stride, y_stride, yaw_stride, inc, foot_height)
                if not success:
                    self.reset_hexapod()
                    return False

                aug_inc = abs(inc - self.inc_prev)
                temp_point = [aug_inc * x_stride, aug_inc * y_stride, 0]
                world_point = np.dot(self.T_sf[:3, :3], temp_point)
                self.T_sf[:3, 3] += world_point
                rpy = ang.rotation_matrix_to_euler_angles(self.T_sf[:3, :3])
                rpy[2] += aug_inc * yaw_stride
                self.T_sf[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
                self.core.pub_group.publish(self.hexapod_command)
                self.update_tsf_transform(mp)

                self.inc_prev = inc
                self.step_cntr += 1.0
                rate.sleep()
        return True

    def tripod_gait(
        self,
        x_stride: float,
        y_stride: float,
        yaw_stride: float,
        inc: int,
        foot_height: float,
    ) -> bool:
        """
        Make the hexapod walk using a tripod gait.

        :param x_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's X-axis
        :param y_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's Y-axis
        :param yaw_stride: desired positive/negative distance to cover in a gait cycle around the
            base_footprint's Z-axis
        :param inc: output from the 'first' sinusoidal function (as described in the Notes above)
            that describes the desired 'x' and 'y' position for a given leg's foot
        :param foot_height: output from the 'second' sinusoidal function (as described in the Notes
            above) that describes the desired 'z' position for a given leg's foot
        :return: `True` if function completed successfully; `False` otherwise
        """
        x_inc = inc * x_stride
        y_inc = inc * y_stride
        yaw_inc = inc * yaw_stride

        for leg in self.leg_list:
            new_point = []
            T_osc = np.identity(4)
            if (leg == 'right_front' or leg == 'right_back' or leg == 'left_middle'):
                T_osc[:3, :3] = ang.euler_angles_to_rotation_matrix([0, 0, -yaw_inc])
                p_f = [-x_inc, -y_inc, 0 if self.step_cntr < self.num_steps/2.0 else foot_height]
            else:
                T_osc[:3, :3] = ang.euler_angles_to_rotation_matrix([0, 0, yaw_inc])
                p_f = [x_inc, y_inc, 0 if self.step_cntr > self.num_steps/2.0 else foot_height]
            T_osc[:3, 3] = p_f
            new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
            success = self.update_joint_command(new_point[:3], leg)
            if not success:
                return False
        return True

    def ripple_gait(
        self,
        x_stride: float,
        y_stride: float,
        yaw_stride: float,
        inc: int,
        foot_height: float,
    ) -> bool:
        """
        Make the hexapod walk using a ripple gait.

        :param x_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's X-axis
        :param y_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's Y-axis
        :param yaw_stride: desired positive/negative distance to cover in a gait cycle around the
            base_footprint's Z-axis
        :param inc: output from the 'first' sinusoidal function (as described in the Notes above)
            that describes the desired 'x' and 'y' position for a given leg's foot
        :param foot_height: output from the 'second' sinusoidal function (as described in the Notes
            above) that describes the desired 'z' position for a given leg's foot
        :return: `True` if function completed successfully; `False` otherwise
        """
        for pair in self.ripple_leg_pairs:
            z_inc = 0
            new_point = []
            T_osc = np.identity(4)
            if pair != self.ripple_leg_pairs[0]:
                self.ripple_incs[pair] -= abs(inc - self.inc_prev)
            else:
                self.ripple_incs[pair] += abs(inc - self.inc_prev) * 2.0
                z_inc = foot_height
            x_inc = self.ripple_incs[pair] * x_stride
            y_inc = self.ripple_incs[pair] * y_stride
            yaw_inc = self.ripple_incs[pair] * yaw_stride
            p_f = [x_inc, y_inc, z_inc]
            T_osc[:3, 3] = p_f
            T_osc[:3, :3] = ang.euler_angles_to_rotation_matrix([0, 0, yaw_inc])
            for leg in self.ripple_legs[pair]:
                new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
                success = self.update_joint_command(new_point[:3], leg)
                if not success:
                    self.ripple_leg_pairs = ['first', 'second', 'third']
                    self.ripple_incs = {p: 0 for p in self.ripple_leg_pairs}
                    self.period_cntr = 0
                    return False
        self.period_cntr += 1.0
        if (self.period_cntr == self.num_steps/2.0):
            old_pair = self.ripple_leg_pairs.pop(0)
            self.ripple_leg_pairs.append(old_pair)
            self.period_cntr = 0
        return True

    def wave_gait(
        self,
        x_stride: float,
        y_stride: float,
        yaw_stride: float,
        inc: int,
        foot_height: float,
    ) -> bool:
        """
        Make the hexapod walk using a wave gait.

        :param x_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's X-axis
        :param y_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's Y-axis
        :param yaw_stride: desired positive/negative distance to cover in a gait cycle around the
            base_footprint's Z-axis
        :param inc: output from the 'first' sinusoidal function (as described in the Notes above)
            that describes the desired 'x' and 'y' position for a given leg's foot
        :param foot_height: output from the 'second' sinusoidal function (as described in the Notes
            above) that describes the desired 'z' position for a given leg's foot
        :return: `True` if function completed successfully; `False` otherwise
        """
        for leg in self.wave_legs:
            z_inc = 0
            new_point = []
            T_osc = np.identity(4)
            if leg != self.wave_legs[0]:
                self.wave_incs[leg] -= abs(inc - self.inc_prev)
            else:
                self.wave_incs[leg] += abs(inc - self.inc_prev) * 5.0
                z_inc = foot_height
            x_inc = self.wave_incs[leg] * x_stride
            y_inc = self.wave_incs[leg] * y_stride
            yaw_inc = self.wave_incs[leg] * yaw_stride
            p_f = [x_inc, y_inc, z_inc]
            T_osc[:3, 3] = p_f
            T_osc[:3, :3] = ang.euler_angles_to_rotation_matrix([0, 0, yaw_inc])
            new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
            success = self.update_joint_command(new_point[:3], leg)
            if not success:
                self.wave_legs = [
                    'right_front', 'left_front', 'right_middle',
                    'left_middle', 'right_back', 'left_back'
                ]
                self.wave_incs = {this_leg: 0 for this_leg in self.wave_legs}
                self.period_cntr = 0
                return False
        self.period_cntr += 1.0
        if (self.period_cntr == self.num_steps/2.0):
            old_leg = self.wave_legs.pop(0)
            self.wave_legs.append(old_leg)
            self.period_cntr = 0
        return True

    def move_in_world_rough(
        self,
        x_stride: float = 0.0,
        y_stride: float = 0.0,
        yaw_stride: float = 0.0,
        max_foot_height: float = 0.02,
        leg_up_time: float = 0.5,
        num_swing_steps: float = 10.0,
        mp: float = 0.150,
        ap: float = 0.075,
        leg_down_inc: float = 0.001,
        threshold: int = 70,
        reset_foot_points: bool = False,
        reset_height: float = 0.12,
        num_cycles: int = 1,
        cycle_freq: float = 20.0
    ) -> bool:
        """
        Move the hexapod 'base_footprint' frame relative to the 'odom' frame in uneven terrain.

        This method uses the tripod gait.

        :param x_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's X-axis
        :param y_stride: desired positive/negative distance to cover in a gait cycle relative to
            the base_footprint's Y-axis
        :param yaw_stride: desired positive/negative distance to cover in a gait cycle around the
            base_footprint's Z-axis
        :param max_foot_height: max height [meters] that a leg's foot will be lifted during the
            'swing' phase
        :param leg_up_time: time in seconds that it should take for a foot to move up to
            'max_foot_height'
        :param num_swing_steps: number of iterations to complete planar motion w.r.t. the XY plane
        :param mp: time [sec] that each joint should spend moving per step in num_swing_steps
        :param ap: time [sec] that each joint should spend moving per step in num_swing_steps
        :param leg_down_inc: length [meters] that a leg moves down every iteration
        :param threshold: the femur motor current [mA] above 0 that is considered a 'ground touch'
            for the leg
        :param reset_foot_points: set to `True` to reset 'self.foot_points' to
            'self.home_foot_points' before moving the hexapod
        :param reset_height: if resetting foot points, this sets the 'z' value of self.T_fb
        :param num_cycles: number of gait cycles to complete before exiting
        :param cycle_freq: frequency at which the gait cycle should run
        :return: `True` if function completed successfully; `False` otherwise
        """
        if reset_foot_points:
            self.foot_points = copy.deepcopy(self.home_foot_points)
            self.move_in_place(z=reset_height)

        # setup 'tripod' gait variables
        first_set = ['left_front', 'left_back', 'right_middle']
        second_set = ['right_front', 'right_back', 'left_middle']
        sets_to_cycle = [first_set, second_set]
        rate = self.core.create_rate(frequency=cycle_freq)
        for _ in range(num_cycles):
            for this_set in sets_to_cycle:
                # Move all legs in a set up to 'max_foot_height'
                for leg in this_set:
                    new_point = np.r_[self.foot_points[leg][:2], max_foot_height]
                    success = self.update_joint_command(new_point, leg)
                    if not success:
                        return False
                    self.foot_points[leg][2] = max_foot_height
                self.set_trajectory_time('all', leg_up_time, leg_up_time/2.0)
                self.core.pub_group.publish(self.hexapod_command)
                time_start = self.core.get_clock().now()

                # Move all legs in the XY plane according to the 'stride' parameters
                self.set_trajectory_time('all', mp, ap)
                time_diff = leg_up_time - (self.core.get_clock().now() - time_start)
                if time_diff > 0:
                    time.sleep(time_diff)
                inc_prev = 0
                for step in range(1, int(num_swing_steps) + 1):
                    inc = 0.25*(1 + math.sin(np.pi*(step/num_swing_steps) - np.pi/2))
                    x_inc = inc * x_stride
                    y_inc = inc * y_stride
                    yaw_inc = inc * yaw_stride
                    for leg, point in self.foot_points.items():
                        T_osc = np.identity(4)
                        if leg not in set:
                            T_osc[:3, :3] = ang.euler_angles_to_rotation_matrix([0, 0, -yaw_inc])
                            p_f = [-x_inc, -y_inc, 0]
                        else:
                            T_osc[:3, :3] = ang.euler_angles_to_rotation_matrix([0, 0, yaw_inc])
                            p_f = [x_inc, y_inc, 0]
                        T_osc[:3, 3] = p_f
                        new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
                        success = self.update_joint_command(new_point[:3], leg)
                        if not success:
                            return False
                        if step == num_swing_steps:
                            self.foot_points[leg] = list(new_point[:3])
                    aug_inc = abs(inc - inc_prev)
                    temp_point = [aug_inc * x_stride, aug_inc * y_stride, 0]
                    world_point = np.dot(self.T_sf[:3, :3], temp_point)
                    self.T_sf[:3, 3] += world_point
                    rpy = ang.rotation_matrix_to_euler_angles(self.T_sf[:3, :3])
                    rpy[2] += aug_inc * yaw_stride
                    self.T_sf[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
                    self.core.pub_group.publish(self.hexapod_command)
                    self.update_tsf_transform(mp)
                    inc_prev = inc
                    rate.sleep()

                # Move all legs in a set down until 'ground touch' is achieved
                all_feet_grounded = False
                current_foot_height = max_foot_height
                time_start = self.core.get_clock().now()
                while not all_feet_grounded:
                    all_feet_grounded = True
                    current_foot_height -= leg_down_inc
                    for leg in set:
                        if (self.core.get_clock().now() <= time_start + 0.6):
                            new_point = np.r_[self.foot_points[leg][:2], current_foot_height]
                            success = self.update_joint_command(new_point, leg)
                            if not success:
                                continue
                            self.foot_points[leg][2] = current_foot_height
                            all_feet_grounded = False
                        elif (self.core.get_clock().now() > time_start + 0.6):
                            joint_effort = self.core.joint_states.effort[
                                self.core.js_index_map[f'{leg}_femur']
                            ]
                            if (joint_effort < threshold):
                                new_point = np.r_[self.foot_points[leg][:2], current_foot_height]
                                success = self.update_joint_command(new_point, leg)
                                if not success:
                                    continue
                                self.foot_points[leg][2] = current_foot_height
                                all_feet_grounded = False
                    self.core.pub_group.publish(self.hexapod_command)
                    rate.sleep()
        return True

    def get_odometry(self) -> List[float]:
        """
        Get current odometry from `<robot_name>/odom` to `<robot_name>/base_footprint`.

        :return: 3-element-list containing the 2D pose of the hexapod (as [x, y, theta])
        """
        rpy = ang.rotation_matrix_to_euler_angles(self.T_sf[:3, :3])
        return [self.T_sf[0, 3], self.T_sf[1, 3], rpy[2]]

    def get_body_pose(self):
        """
        Get current body pose (i.e. `self.T_fb`).

        :return: 4x4 transformation matrix copy of `self.T_fb`
        """
        return np.array(self.T_fb)

    def get_foot_points(self) -> List[Dict[str, List[float]]]:
        """
        Get all six hexapod initial foot points.

        Foot points are the foot/feet location before beginning a gait cycle

        :param foot_points: dictionary containing all six hexapod foot points
        :details: each foot point is the [x, y, z] location of the 'foot_link' frame relative to
            the 'base_footprint' frame; the keys to the dictionary are the 'leg' names in
            'self.leg_list'
        """
        return copy.deepcopy(self.foot_points)

    def set_foot_points(self, leg: str, foot_points: List[float]) -> None:
        """
        Set one or all initial hexapod foot points.

        Foot points are the foot/feet location before beginning a gait cycle

        :param leg: one of the legs defined in 'self.leg_list' or 'all' if passing in a dictionary
            containg all desired foot points
        :param foot_points: a list containing the [x, y, z] location of the 'foot_link' frame for
            the given leg relative to the 'base_footprint' frame OR a dictionary containing each
            leg's location of the 'foot_link' frame relative to the 'base_footprint' frame (keys
            are the leg names defined in 'self.leg_list')
        """
        if leg == 'all':
            self.foot_points = copy.deepcopy(foot_points)
        else:
            self.foot_points[leg] = copy.deepcopy(foot_points)

    def set_home_foot_points(self, leg: str, foot_points: List[float]) -> None:
        """
        Set one or all initial hexapod home foot points.

        Foot points are the foot/feet location before beginning a gait cycle

        :param leg: one of the legs defined in 'self.leg_list' or 'all' if passing in a dictionary
            containg all desired foot points
        :param foot_points: a list containing the [x, y, z] location of the 'foot_link' frame for
            the given leg relative to the 'base_footprint' frame OR a dictionary containing each
            leg's location of the 'foot_link' frame relative to the 'base_footprint' frame (keys
            are the leg names defined in 'self.leg_list')
        """
        if leg == 'all':
            self.home_foot_points = copy.deepcopy(foot_points)
        else:
            self.home_foot_points[leg] = copy.deepcopy(foot_points)

    def set_home_height(self, height: float) -> None:
        """Set the home height."""
        self.home_height = height
