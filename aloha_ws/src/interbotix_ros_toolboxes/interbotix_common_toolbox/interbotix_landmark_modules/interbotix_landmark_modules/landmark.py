# Copyright 2023 Trossen Robotics
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

from __future__ import annotations

from typing import Callable, Dict, List, Union

from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class Landmark:
    """
    Module that describes a landmark.

    Stores data about landmarks, calculate transforms, and publish that information to the
        static TF tree.
    """

    def __init__(
        self,
        label: str,
        tag_id: int,
        node_inf: Node,
        tf_buffer: tf2_ros.Buffer = None,
        tf_listener: tf2_ros.TransformListener = None,
        callback: Callable = None,
        landmark_ns: str = 'landmarks',
    ) -> None:
        """
        Construct a Landmark object.

        :param label: name of this tracked Landmark
        :param tag_id: tag_id of this tracked Landmark
        :param node_inf: reference to the rclpy.node.Node on which to build this interface
        :param landmark_ns: (optional) namespace where the ROS parameters needed by the module are
            located. Defaults to "landmarks".
        """
        self.node_inf = node_inf

        # tf buffer and listener
        self.tf_buffer = tf_buffer if tf_buffer is not None else tf2_ros.Buffer(node=node_inf)
        self.listener = tf_listener if tf_listener is not None else tf2_ros.TransformListener(
            buffer=self.tf_buffer, node=self.node_inf
        )

        self._label = label
        self._tag_id = tag_id
        self._callback = callback
        self._landmark_ns = landmark_ns

        # transforms
        self._tf_wrt_cam = TransformStamped()
        self._tf_wrt_cam.child_frame_id = self._label
        self._tf_wrt_map = TransformStamped()
        self._tf_wrt_map.child_frame_id = self._label
        self._mounted_offset = 0.0

        # flags
        self._tf_set = False
        self._mounted = False

        self._init_markers()

    @property
    def tag_id(self) -> int:
        """
        Get tag id.

        :return: this Landmark's tag_id
        """
        return self._tag_id

    def transform_to_new_frame(self, parent_old: str, parent_new: str) -> PoseStamped:
        """
        Transform frame from one fixed pose to another.

        :param parent_old: current fixed frame
        :param parent_new: desired fixed frame
        :return: pose in new fixed frame
        """
        tf_old = PoseStamped()
        tf_old.pose.position.x = self._tf_wrt_cam.transform.translation.x
        tf_old.pose.position.y = self._tf_wrt_cam.transform.translation.y
        tf_old.pose.position.z = self._tf_wrt_cam.transform.translation.z
        tf_old.pose.orientation.x = self._tf_wrt_cam.transform.rotation.x
        tf_old.pose.orientation.y = self._tf_wrt_cam.transform.rotation.y
        tf_old.pose.orientation.z = self._tf_wrt_cam.transform.rotation.z
        tf_old.pose.orientation.w = self._tf_wrt_cam.transform.rotation.w
        tf_old.header.frame_id = parent_old
        tf_old.header.stamp = self.node_inf.get_clock().now().to_msg()
        try:
            # transform pose to frame parent_new
            tf_new = self.tf_buffer.transform(
                object_stamped=tf_old,
                target_frame=parent_new,
            )
            # set/update tf with respect to map
            return tf_new
        except TransformException as e:
            self.node_inf.get_logger().error(
                f"Could not transform between frames '{parent_new}' and '{parent_old}': {e}",
            )
            raise e

    @property
    def label(self) -> str:
        """
        Get label.

        :return: label
        """
        return self._label

    @label.setter
    def label(self, label: str) -> None:
        """
        Set label.

        :param label: new label for this landmark
        """
        self._label = label

    @property
    def x(self) -> float:
        """
        Get landmark x position with respect to map frame.

        :return: landmark x position with respect to map frame
        :details: returns 0.0 if the TF of the landmark has not been set
        """
        if self._tf_set:
            return self._tf_wrt_map.transform.translation.x
        else:
            return 0.0

    @property
    def y(self) -> float:
        """
        Get landmark y position with respect to map frame.

        :return: landmark y position with respect to map frame
        :details: returns 0.0 if the TF of the landmark has not been set
        """
        if self._tf_set:
            return self._tf_wrt_map.transform.translation.y
        else:
            return 0.0

    @property
    def theta(self) -> float:
        """
        Get landmark yaw rotation with respect to map frame.

        :return: landmark yaw rotation with respect to map frame
        :details: returns 0.0 if the TF of the landmark has not been set
        """
        if self._tf_set:
            q = self._tf_wrt_map.transform.rotation
            return euler_from_quaternion(
                [q.x, q.y, q.z, q.w],
                'rxyz'
            )[2]
        else:
            return 0.0

    @property
    def tf_wrt_cam(self) -> TransformStamped:
        """
        Get for transform with respect to camera.

        :return: pose transform with respect to camera
        """
        return self._tf_wrt_cam

    @tf_wrt_cam.setter
    def tf_wrt_cam(self, tf: Union[Transform, TransformStamped]) -> None:
        """
        Set for transform with respect to camera frame.

        :param tf: translation and orientation from cam to landmark
        """
        if isinstance(tf, Transform):
            self._tf_wrt_cam.transform.translation = tf.translation
            self._tf_wrt_cam.transform.rotation = tf.rotation
        elif isinstance(tf, TransformStamped):
            self._tf_wrt_cam.transform.translation = tf.transform.translation
            self._tf_wrt_cam.transform.rotation = tf.transform.rotation
        else:
            raise TypeError(
                'Must be of type geometry_msgs.msg.Transform or '
                f'geometry_msgs.msg.TransformStamped, was type {type(tf)}.'
            )

    @property
    def cam_frame_id(self) -> str:
        """
        Get transform with respect to camera frame frame id.

        :return: The frame_id the transform is relative to
        """
        return self._tf_wrt_cam.header.frame_id

    @cam_frame_id.setter
    def cam_frame_id(self, frame_id: str) -> None:
        """
        Set transform with respect to camera frame frame id.

        :param frame_id: The frame_id the transform should be relative to
        """
        self._tf_wrt_cam.header.frame_id = frame_id

    def get_tf_wrt_map(self) -> TransformStamped:
        """
        Get transform with respect to map frame.

        :return: transform with respect to map frame
        """
        return self._tf_wrt_map

    def set_tf_wrt_map(self, pose: Union[Pose, PoseStamped], parent_new: str = 'map') -> None:
        """
        Set transform with respect to map frame.

        :param pose: translation and orientation from cam to landmark
        """
        if isinstance(pose, Pose):
            self._tf_wrt_map.transform.translation.x = pose.position.x
            self._tf_wrt_map.transform.translation.y = pose.position.y
            self._tf_wrt_map.transform.translation.z = pose.position.z
            self._tf_wrt_map.transform.rotation.x = pose.orientation.x
            self._tf_wrt_map.transform.rotation.y = pose.orientation.y
            self._tf_wrt_map.transform.rotation.z = pose.orientation.z
            self._tf_wrt_map.transform.rotation.w = pose.orientation.w
        elif isinstance(pose, PoseStamped):
            self._tf_wrt_map.transform.translation.x = pose.pose.position.x
            self._tf_wrt_map.transform.translation.y = pose.pose.position.y
            self._tf_wrt_map.transform.translation.z = pose.pose.position.z
            self._tf_wrt_map.transform.rotation.x = pose.pose.orientation.x
            self._tf_wrt_map.transform.rotation.y = pose.pose.orientation.y
            self._tf_wrt_map.transform.rotation.z = pose.pose.orientation.z
            self._tf_wrt_map.transform.rotation.w = pose.pose.orientation.w
        else:
            raise TypeError(
                'Must be of type geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped, '
                f'was {type(pose)}.'
            )
        self._tf_wrt_map.header.stamp = self.node_inf.get_clock().now().to_msg()
        self._tf_wrt_map.header.frame_id = parent_new

    def update_tf(self, parent_old: str, parent_new: str) -> None:
        """
        Update the transformation to fixed frame.

        :param parent_old: current frame
        :param parent_old: new frame
        :details: successfully calling this function will set the TF of this landmark
        """
        self.set_tf_wrt_map(
            pose=self.transform_to_new_frame(parent_old, parent_new),
            parent_new=parent_new
        )
        self._tf_set = True

    @property
    def mounted(self) -> bool:
        """
        Get the mounted status of this landmark.

        :return: `True` if this landmark is mounted; `False` otherwise
        """
        return self._mounted

    @mounted.setter
    def mounted(self, mounted: bool) -> None:
        """
        Set mounted status of this landmark.

        :param mounted: boolean value to set the mounted status of this landmark to
        """
        self._mounted = mounted

    @property
    def mounted_offset(self) -> float:
        """
        Get the mounted offset of this landmark.

        :return: the mounted offset of this landmark.
        """
        return self._mounted_offset

    @mounted_offset.setter
    def mounted_offset(self, offset: float = 0.0) -> None:
        """
        Set the mounted offset of this landmark.

        :param offset: (optional) the offset of this landmark. Defaults to 0.0
        """
        self._mounted_offset = offset

    @property
    def nav_goal(self) -> List[float]:
        """
        Get for the [x, y, theta] navigation goal for this landmark.

        :return: list containing [x, y, theta] navigation goal for this landmark
        """
        if not self.mounted:  # know offset is 0, can just return the [x, y, theta]
            return [self.x, self.y, self.theta]
        else:
            tf_goal = TransformStamped()
            tf_goal.header.stamp = Time()
            tf_goal.header.frame_id = self.label
            tf_goal.child_frame_id = f'{self.label}_goal'

            # z-axis is normal to tag face, 'front of tag'
            tf_goal.transform.translation.z = self.mounted_offset
            tf_goal.transform.rotation.w = 1.0  # ensure valid quaternion
            self.tf_buffer.set_transform(
                transform=tf_goal,
                authority='default_authority'
            )

            # get tf from map to goal frame [x, y, theta]
            tf_map_to_goal = self.tf_buffer.lookup_transform(
                target_frame=self._landmark_ns,
                source_frame=tf_goal.child_frame_id,
                time=Time()
            )

            return [
                tf_map_to_goal.transform.translation.x,
                tf_map_to_goal.transform.translation.y,
                euler_from_quaternion([
                    tf_map_to_goal.transform.rotation.x,
                    tf_map_to_goal.transform.rotation.y,
                    tf_map_to_goal.transform.rotation.z,
                    tf_map_to_goal.transform.rotation.w
                ])[2]
            ]

    def _init_markers(self) -> None:
        """Initialize marker values."""
        # spherical marker (location)
        self.marker_sphere = Marker()
        self.marker_sphere.id = 0
        self.marker_sphere.header.frame_id = self._landmark_ns
        self.marker_sphere.header.stamp = Time().to_msg()
        self.marker_sphere.ns = self.label
        self.marker_sphere.type = Marker.SPHERE
        self.marker_sphere.action = Marker.ADD
        self.marker_sphere.scale.x = 0.025
        self.marker_sphere.scale.y = 0.025
        self.marker_sphere.scale.z = 0.025
        self.marker_sphere.color.a = 1.0
        self.marker_sphere.color.r = 1.0
        self.marker_sphere.color.g = 0.0
        self.marker_sphere.color.b = 0.0
        self.marker_sphere.pose.position.z = 0.0

        # text marker (label)
        self.marker_text = Marker()
        self.marker_text.id = 1
        self.marker_text.header.frame_id = self._landmark_ns
        self.marker_text.header.stamp = Time().to_msg()
        self.marker_text.ns = self.label
        self.marker_text.type = Marker.TEXT_VIEW_FACING
        self.marker_text.text = f'{self.label}_goal'
        self.marker_text.scale.x = 0.05
        self.marker_text.scale.y = 0.05
        self.marker_text.scale.z = 0.05
        self.marker_text.color.a = 1.0
        self.marker_text.color.r = 1.0
        self.marker_text.color.g = 1.0
        self.marker_text.color.b = 1.0
        self.marker_text.pose.position.z = 0.05

    def __eq__(self, other: Landmark) -> bool:
        if isinstance(other, Landmark):
            return (
                self.tag_id == other.tag_id and
                self.label == other.label and
                self.x == other.x and
                self.y == other.y and
                self.theta == other.theta
            )
        else:
            raise TypeError(f'Must be of type Landmark, was {type(other)}.')

    def __repr__(self) -> str:
        tf = self._tf_wrt_map if self._tf_set else self.tf_wrt_cam
        return (
            'interbotix_landmark_modules.landmark.Landmark('
            f'label={self.label},'
            f'tag_id={self.tag_id},'
            f'tf_set={self._tf_set},'
            f'mounted={self.mounted},'
            f'mounted_offset={self.mounted_offset},'
            f'tf={tf}'
            ')'
        )


class LandmarkCollection:
    """Class to interact with Landmark data structure."""

    static_tf_pub: Publisher
    marker_pub: Publisher

    def __init__(
        self,
        node_inf: Node,
        landmarks: Dict[int, Landmark] = {},
        observation_frame: str = None,
        fixed_frame: str = None,
        ros_on: bool = False,
        tf_buffer: tf2_ros.Buffer = None,
        tf_listener: tf2_ros.TransformListener = None,
    ):
        """
        Construct LandmarkCollection object.

        :param node_inf: reference to the rclpy.node.Node on which to build this interface
        :param landmarks: (optional): A dictionary of ints to Landmarks. Defaults to an empty
            dictionary.
        :param observation_frame: (optional): Frame that the Landmarks will be observed from;
            typically the camera's optical frame. Defaults to `None`.
        :param fixed_frame: (optional): Fixed frame that the Landmarks should be published relative
            to. Defaults to `None`.
        :param ros_on: (optional): Whether or not ROS is on. Defaults to `False`.
        """
        self._node_inf = node_inf
        self._landmarks = landmarks

        # tf buffer and listener
        self.tf_buffer = tf_buffer if tf_buffer is not None else tf2_ros.Buffer(
            node=self._node_inf
        )
        self.listener = tf_listener if tf_listener is not None else tf2_ros.TransformListener(
            buffer=self.tf_buffer, node=self._node_inf
        )

        # if we will use ROS, get params and set up pubs/subs
        if ros_on:
            self.ROS = True

            self.observation_frame = self._node_inf.get_parameter_or(
                'observation_frame',
                observation_frame,
            )
            self.fixed_frame = self._node_inf.get_parameter_or(
                'fixed_frame',
                fixed_frame,
            )

            self.static_tf_pub = self._node_inf.create_publisher(
                TransformStamped,
                'static_transforms',
                qos_profile=10,
            )

            self.marker_pub = self._node_inf.create_publisher(
                MarkerArray,
                'landmark_markers',
                qos_profile=10,
            )

        else:
            self.ROS = False
            self.observation_frame = observation_frame
            self.fixed_frame = fixed_frame

    def get_landmark(self, tag_id: int) -> Landmark:
        """
        Get the specified landmark corresponding to the tag_id.

        :return: landmark with specified tag_id
        """
        return self._landmarks.get(tag_id)

    def get_landmarks(self, tag_ids: Union[List[int], None] = None) -> List[Landmark]:
        """
        Get the specified landmarks corresponding to their tag_ids.

        :return: landmarks with specified label
        :details: If tag_ids is not given, returns all landmarks as dict_values. If tag_ids is a
            list of strs, returns landmarks corresponding to the tag_ids in that list
        """
        if tag_ids is None:
            return list(self._landmarks.values())
        elif isinstance(tag_ids, list):
            return [self._landmarks.get(label) for label in tag_ids]
        else:
            raise TypeError(f'Must be of type List[int] or None, was {type(tag_ids)}')

    def add_landmark(self, label: str, tag_id: int):
        """
        Add landmark to data dictionary.

        :param label: label for added landmark to add
        :param tag_id: tag_id for added landmark and dictionary key
        """
        self._landmarks.update({
            tag_id: Landmark(
                label=label,
                tag_id=tag_id,
                node_inf=self._node_inf,
                tf_listener=self.listener,
                tf_buffer=self.tf_buffer,
            )
        })

    def pop_landmark(self, tag_id: int) -> Landmark:
        """
        Remove Landmark from data dictionary.

        :param tag_id: tag_id of Landmark to remove
        :return: Removed Landmark
        """
        return self._landmarks.pop(tag_id)

    def get_valid_tags(self) -> List[int]:
        """
        Return the tag_ids of the tags of any Landmark in the Collection.

        :return: list of tag_ids corresponding to landmarks in the Collection
        :details: updates the list of valid tags before returning
        """
        self.update_valid_tags()
        return self.valid_tags

    def update_valid_tags(self) -> None:
        """Update the list of valid tags."""
        self.valid_tags = [lm.tag_id for lm in self.get_landmarks()]

    def get_set_tags(self) -> List[int]:
        """
        Return the list of seen tags.

        :return: list of ids corresponding to seen landmarks in the Collection
        """
        return [lm.tag_id for lm in self.get_landmarks() if lm._tf_set]

    def get_set_landmarks(self) -> List[Landmark]:
        """
        Return the list of seen landmarks.

        :return: list of seen landmarks in the Collection
        """
        return [lm for lm in self.get_landmarks() if lm._tf_set]

    def save(self, filepath, ids: Union[List[int], None] = None) -> bool:
        """
        Save landmarks to a specified yaml file.

        :param filepath: absolute filepath to save structured landmark data to
        :param ids: (optional) IDs of the landmarks to save. If set to `None`, saves all Landmarks.
            Defaults to `None`
        :return: `True` if saved successfully, `False` otherwise
        :details: if the collection is empty, this method will always return `True`
        """
        if self.is_empty():
            return True

        lm_yaml = {}
        for lm in self.get_landmarks(ids):
            tf_map = lm._tf_wrt_map
            lm_dict = {}
            lm_dict['id'] = lm.tag_id
            lm_dict['label'] = lm._label
            lm_dict['set'] = lm._tf_set
            lm_dict['mounted'] = lm.mounted
            lm_dict['mounted_offset'] = lm._mounted_offset
            if lm._tf_set:
                lm_dict['tf'] = {}
                lm_dict['tf']['frame_id'] = tf_map.header.frame_id
                lm_dict['tf']['child_frame_id'] = tf_map.child_frame_id
                lm_dict['tf']['x'] = float(tf_map.transform.translation.x)
                lm_dict['tf']['y'] = float(tf_map.transform.translation.y)
                lm_dict['tf']['z'] = float(tf_map.transform.translation.z)
                lm_dict['tf']['qx'] = float(tf_map.transform.rotation.x)
                lm_dict['tf']['qy'] = float(tf_map.transform.rotation.y)
                lm_dict['tf']['qz'] = float(tf_map.transform.rotation.z)
                lm_dict['tf']['qw'] = float(tf_map.transform.rotation.w)
            lm_yaml[lm.tag_id] = lm_dict

        try:
            with open(filepath, 'w') as yamlfile:
                yaml.dump(lm_yaml, yamlfile, default_flow_style=False)
            self._node_inf.get_logger().info(f"Saved landmarks to '{filepath}'.")
        except IOError as e:
            self._node_inf.get_logger().error(f"Could not save landmarks to '{filepath}': {e}")
            return False

        return True

    def load(self, filepath: str) -> bool:
        """
        Load landmarks from a specified yaml file.

        :param filepath: path to yaml file containing structured landmark data
        :return: `True` if loaded successfully, `False` otherwise
        """
        lm_dict: Dict[str, Dict]
        try:
            with open(filepath, 'r') as yamlfile:
                lm_dict = yaml.safe_load(yamlfile)

            if lm_dict is None:
                self._node_inf.get_logger().warning(f"No landmarks found in file '{filepath}'.")
                return True

            for key in lm_dict.keys():

                self.add_landmark(
                    label=lm_dict[key]['label'],
                    id_num=key)

                # get transform from configs
                if 'tf' in lm_dict[key].keys():
                    self._landmarks[key]._tf_wrt_map = TransformStamped()
                    self._landmarks[key]._tf_wrt_map.header.stamp = (
                        self._node_inf.get_clock().now().to_msg()
                    )
                    if lm_dict[key]['tf']['frame_id'] == '':
                        self._landmarks[key]._tf_wrt_map.header.frame_id = self.observation_frame
                    else:
                        self._landmarks[key]._tf_wrt_map.header.frame_id = (
                            lm_dict[key]['tf']['frame_id']
                        )
                    self._landmarks[key]._tf_wrt_map.child_frame_id = (
                        lm_dict[key]['tf']['child_frame_id']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.translation.x = (
                        lm_dict[key]['tf']['x']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.translation.y = (
                        lm_dict[key]['tf']['y']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.translation.z = (
                        lm_dict[key]['tf']['z']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.rotation.x = (
                        lm_dict[key]['tf']['qx']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.rotation.y = (
                        lm_dict[key]['tf']['qy']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.rotation.z = (
                        lm_dict[key]['tf']['qz']
                    )
                    self._landmarks[key]._tf_wrt_map.transform.rotation.w = (
                        lm_dict[key]['tf']['qw']
                    )
                    self._landmarks[key]._tf_set = True
                else:
                    self._landmarks[key]._tf_wrt_cam.header.frame_id = self.observation_frame
                    self._landmarks[key]._tf_wrt_cam.child_frame_id = self._landmarks[key].label
                    self._landmarks[key]._tf_set = False

                # get mounted/mounted_offset values from configs
                if 'mounted' in lm_dict[key].keys():
                    self._landmarks[key].mounted = lm_dict[key]['mounted']
                    self._landmarks[key].mounted_offset = lm_dict[key]['mounted_offset']

            self.update_valid_tags()
            if self.ROS:
                self.pub_tfs(self.valid_tags)
                rclpy.spin_once(self._node_inf, timeout_sec=0.1)
                self.pub_markers(self.valid_tags)
            return True

        except IOError:
            self._node_inf.get_logger().warning(
                f"File at '{filepath}' does not exist yet. No landmarks loaded."
            )
            return False

    def is_empty(self) -> bool:
        """
        Check if landmarks is empty.

        :return: `True` if landmarks is empty, `False` otherwise
        """
        return len(self._landmarks) == 0

    def pub_tfs(self, tag_ids: Union[List[int], int, None] = None) -> None:
        """
        Publish TFs to static transforms.

        :param tag_ids: list of tags to publish. Publishes all set Landmarks if `None`. Defaults to
            `None`
        """
        if self.is_empty():
            return True

        if isinstance(tag_ids, int):
            tag_ids = [tag_ids]

        if tag_ids is None:
            # when no tags are specified, pubs all
            for lm in self.get_landmarks():
                if lm._tf_set:
                    self.static_tf_pub.publish(
                        lm._tf_wrt_map
                    )
        else:
            # when list of tags is specified, pubs tags in list
            for lm in [self.get_landmark(tag_id) for tag_id in tag_ids]:
                if lm._tf_set:
                    self.static_tf_pub.publish(
                        lm._tf_wrt_map
                    )
                    print(f'published lm {lm.label}')

    def update_markers(self) -> None:
        """Update markers."""
        for lm in self.get_set_landmarks():  # only publish seen tags
            g = lm.nav_goal
            lm.marker_sphere.pose.position.x = g[0]
            lm.marker_sphere.pose.position.y = g[1]
            lm.marker_sphere.pose.position.z = 0.1
            lm.marker_sphere.pose.orientation.x = 0.0
            lm.marker_sphere.pose.orientation.y = 0.0
            lm.marker_sphere.pose.orientation.z = 0.0
            lm.marker_sphere.pose.orientation.w = 1.0
            lm.marker_sphere.header.stamp = Time()

            lm.marker_text.pose.position.x = g[0]
            lm.marker_text.pose.position.y = g[1]
            lm.marker_text.pose.position.z = 0.15
            lm.marker_text.pose.orientation.x = 0.0
            lm.marker_text.pose.orientation.y = 0.0
            lm.marker_text.pose.orientation.z = 0.0
            lm.marker_text.pose.orientation.w = 1.0
            lm.marker_text.header.stamp = Time()

    def pub_markers(self, tag_ids: Union[List[int], int, None] = None) -> None:
        """
        Publish specified markers.

        :param tag_ids: IDs to publish markers of. Will publish all if set to `None`. Defaults to
            `None`
        """
        if not self.ROS:
            self._node_inf.get_logger().warning('Tried to publish marker but node is not active.')
            return
        else:
            self.update_markers()
            msg = MarkerArray()
            for lm in self.get_set_landmarks():
                if tag_ids is None:
                    msg.markers.append(lm.marker_sphere)
                    msg.markers.append(lm.marker_text)
                    self.marker_pub.publish(msg)
                if lm.tag_id in tag_ids:
                    msg.markers.append(lm.marker_sphere)
                    msg.markers.append(lm.marker_text)
                    self.marker_pub.publish(msg)

    def __repr__(self):
        return (
            'interbotix_landmark_modules.landmark.LandmarkCollection('
            f'landmarks={self.get_landmarks()}'
            ')'
        )

    def __len__(self):
        return len(self._landmarks)
