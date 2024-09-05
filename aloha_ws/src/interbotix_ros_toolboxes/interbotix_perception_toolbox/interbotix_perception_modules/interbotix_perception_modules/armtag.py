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

from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from interbotix_common_modules import angle_manipulation as ang
from interbotix_common_modules.common_robot.robot import InterbotixRobotNode
from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface
import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from tf_transformations import euler_from_quaternion, quaternion_from_euler


class InterbotixArmTagInterface:
    """Finds an arm's base link frame relative to some reference frame using arm's AprilTag."""

    def __init__(
        self,
        armtag_ns: str = 'armtag',
        apriltag_ns: str = 'apriltag',
        ref_frame: str = None,
        arm_tag_frame: str = None,
        arm_base_frame: str = None,
        node_inf: InterbotixRobotNode = None,
        args=None,
    ) -> None:
        """
        Construct InterbotixArmTagInterface node.

        :param armtag_ns: namespace where the ROS parameters needed by the module are located
        :param apriltag_ns: namespace where the ROS parameters needed by the
            InterbotixAprilTagInterface module are located
        :param ref_frame: Reference frame from which the image was taken
        :param arm_tag_frame: Frame on the robot that should serve as the arm_tag's frame
        :param arm_base_frame: Frame on the robot that should serve as the base frame
        :param node_inf: reference to the InterbotixRobotNode on which to build the AprilTag
            Interface.
        """
        self.apriltag_inf = InterbotixAprilTagInterface(
            apriltag_ns=apriltag_ns,
            node_inf=node_inf,
            args=args,
        )

        if arm_tag_frame is None:
            self.apriltag_inf.node_inf.declare_parameter(
                'arm_tag_frame',
                'ar_tag_link'
            )
            self.arm_tag_frame = self.apriltag_inf.node_inf.get_parameter(
                'arm_tag_frame').get_parameter_value().string_value
        else:
            self.arm_tag_frame = arm_tag_frame

        if ref_frame is None:
            self.apriltag_inf.node_inf.declare_parameter(
                'ref_frame',
                'camera_color_optical_frame'
            )
            self.ref_frame = self.apriltag_inf.node_inf.get_parameter(
                'ref_frame').get_parameter_value().string_value
        else:
            self.ref_frame = ref_frame

        if arm_base_frame is None:
            self.apriltag_inf.node_inf.declare_parameter(
                'arm_base_frame',
                'base_link'
            )
            self.arm_base_frame = self.apriltag_inf.node_inf.get_parameter(
                'arm_base_frame').get_parameter_value().string_value
        else:
            self.arm_base_frame = arm_base_frame

        self.trans = TransformStamped()
        self.trans.header.frame_id = self.ref_frame
        self.trans.child_frame_id = self.arm_base_frame
        self.trans.transform.rotation.w = 1.0
        self.rpy = [0, 0, 0]

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self.apriltag_inf.node_inf)

        self.apriltag_inf.node_inf.loginfo('Initialized InterbotixArmTagInterface!')

    def find_ref_to_arm_base_transform(
        self,
        ref_frame: str = None,
        arm_base_frame: str = None,
        num_samples: int = 5,
        position_only: bool = False
    ) -> bool:
        """
        Snap AprilTag image, compute transform between robot's base_link and reference frame.

        :param ref_frame: desired reference frame; defaults to self.ref_frame if not specified
        :param arm_base_frame: desired base_link frame (either the arm's actual base_link frame or
            a parent frame); defaults to self.arm_base_frame if not specified
        :param num_samples: number of AprilTag snapshots to take before averaging them to get a
            more accurate pose
        :param position_only: if True, only the x,y,z position of the snapped AR tag will be
            considered; if False, the orientation of the AR tag will be used as well
        :details: the 'position_only' parameter can only be set to True if there already exists a
            'tf' path from the camera color frame to the AR tag frame on the arm; it can be used to
            try to get a more accurate position of the AR tag than what is dictated by the URDF
        """
        if ref_frame is None:
            ref_frame = self.ref_frame
        if arm_base_frame is None:
            arm_base_frame = self.arm_base_frame

        # take the average pose (w.r.t. the camera frame) of the AprilTag over number of samples
        point = Point()
        rpy = [0, 0, 0]
        for _ in range(num_samples):
            ps = self.apriltag_inf.find_pose()
            if ps == Pose():
                return False
            point.x += ps.position.x / float(num_samples)
            point.y += ps.position.y / float(num_samples)
            point.z += ps.position.z / float(num_samples)
            quat_sample = ps.orientation
            quat_list = [quat_sample.x, quat_sample.y, quat_sample.z, quat_sample.w]
            rpy_sample = euler_from_quaternion(quat_list)
            rpy[0] += rpy_sample[0] / float(num_samples)
            rpy[1] += rpy_sample[1] / float(num_samples)
            rpy[2] += rpy_sample[2] / float(num_samples)
        T_CamTag = ang.pose_to_transformation_matrix(
            [point.x, point.y, point.z, rpy[0], rpy[1], rpy[2]]
        )

        # tfBuffer = tf2_ros.Buffer()
        # tf2_ros.TransformListener(tfBuffer, self.apriltag_inf)

        # If position_only, set the orientation of the found AR tag to be equivalent to the
        # orientation of the arm's AR tag as dictated by the URDF
        if (position_only):
            T_CamActualTag = self.get_transform(
                tfBuffer=self.tfBuffer,
                target_frame=self.apriltag_inf.image_frame_id,
                source_frame=self.arm_tag_frame
            )
            T_CamTag[:3, :3] = T_CamActualTag[:3, :3]

        # Now, get a snapshot of the pose of arm's base_link frame w.r.t. the AR tag link (as
        # defined in the URDF - not the one found by the algorithm)
        # We can't publish the AR tag pose found using the AprilTag algorithm to the /tf tree since
        # ROS forbids a link to have multiple parents
        T_TagBase = self.get_transform(
            tfBuffer=self.tfBuffer,
            target_frame=self.arm_tag_frame,
            source_frame=arm_base_frame
        )

        # Now, lets find the transform of the arm's base_link frame w.r.t. the reference frame
        T_CamBase = np.dot(T_CamTag, T_TagBase)
        if ref_frame == self.apriltag_inf.image_frame_id:
            T_RefBase = T_CamBase
        else:
            T_RefCam = self.get_transform(
                tfBuffer=self.tfBuffer,
                target_frame=ref_frame,
                source_frame=self.apriltag_inf.image_frame_id
            )
            T_RefBase = np.dot(T_RefCam, T_CamBase)

        # Now, we can publish the transform from the reference link to the arm's base_link legally
        # as the arm's base_link has no parent (or even if it does, we can safely overwrite it
        # since the 'tf' tree will remain intact)
        self.rpy = ang.rotation_matrix_to_euler_angles(T_RefBase[:3, :3])
        quat = quaternion_from_euler(self.rpy[0], self.rpy[1], self.rpy[2])
        self.trans = TransformStamped()
        self.trans.transform.translation.x = T_RefBase[0, 3]
        self.trans.transform.translation.y = T_RefBase[1, 3]
        self.trans.transform.translation.z = T_RefBase[2, 3]
        self.trans.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.trans.header.frame_id = ref_frame
        self.trans.child_frame_id = arm_base_frame
        self.trans.header.stamp = self.apriltag_inf.node_inf.get_clock().now().to_msg()
        self.apriltag_inf.pub_transforms.publish(self.trans)

        return True

    def get_transform(
        self,
        tfBuffer: tf2_ros.Buffer,
        target_frame: str,
        source_frame: str
    ) -> np.ndarray:
        """
        Lookup a transform and convert it into a 4x4 transformation matrix.

        :param tfBuffer: tf2_ros buffer instance from which to lookup transforms from the 'tf' tree
        :param target_frame: the frame to which data should be transformed
        :param source_frame: the frame where the data originated
        :return: desired 4x4 numpy transformation matrix
        """
        try:
            trans = tfBuffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=Time(),
                timeout=Duration()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:
            self.apriltag_inf.node_inf.logerror(
                f"Failed to look up the transform from '{target_frame}' to '{source_frame}'. {e}"
            )
            return np.identity(4)
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(quat_list)
        return ang.pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])

    def get_x(self) -> float:
        """
        Get the 'x' component of T_RefBase.

        :return: 'x' [m]
        """
        return self.trans.transform.translation.x

    x = property(fget=get_x, doc="The 'x' component of T_RefBase")

    def get_y(self) -> float:
        """
        Get the 'y' component of T_RefBase.

        :return: 'y' [m]
        """
        return self.trans.transform.translation.y

    y = property(fget=get_y, doc="The 'y' component of T_RefBase")

    def get_z(self) -> float:
        """
        Get the 'z' component of T_RefBase.

        :return: 'z' [m]
        """
        return self.trans.transform.translation.z

    z = property(fget=get_z, doc="The 'z' component of T_RefBase")

    def get_roll(self) -> float:
        """
        Get the 'roll' component of T_RefBase.

        :return: 'roll' [rad]
        """
        return self.rpy[0]

    roll = property(fget=get_roll, doc="The 'roll' component of T_RefBase")

    def get_pitch(self) -> float:
        """
        Get the 'pitch' component of T_RefBase.

        :return: 'pitch' [rad]
        """
        return self.rpy[1]

    pitch = property(fget=get_pitch, doc="The 'pitch' component of T_RefBase")

    def get_yaw(self) -> float:
        """
        Get the 'yaw' component of T_RefBase.

        :return: 'yaw' [rad]
        """
        return self.rpy[2]

    yaw = property(fget=get_yaw, doc="The 'yaw' component of T_RefBase")

    def get_parent_frame(self) -> str:
        """
        Get the parent frame of T_RefBase (usually something like 'camera_color_optical_frame').

        :return: 'frame_id'
        """
        return self.trans.header.frame_id

    parent_frame = property(
        fget=get_parent_frame,
        doc="the parent frame of T_RefBase (usually something like 'camera_color_optical_frame')"
    )

    def get_child_frame(self) -> str:
        """
        Get the child frame of T_RefBase (usually something like 'base_link').

        :return: 'child_frame_id'
        """
        return self.trans.child_frame_id

    child_frame = property(
        fget=get_child_frame,
        doc="The child frame of T_RefBase (usually something like 'base_link')"
    )
