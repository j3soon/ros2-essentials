#!/usr/bin/env python3

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

import time
from typing import List, Union

from ament_index_python import get_package_share_directory
from interbotix_common_modules.angle_manipulation import quaternion_is_valid
from interbotix_landmark_modules.landmark import LandmarkCollection
from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface as AprilTag
import rclpy
from rclpy.node import Node


class LandmarkFinder:
    """Application to find landmarks during map exploration."""

    def __init__(self, node_inf: Node):
        """
        Construct the LandmarkFinder.

        :param node_inf: reference to the rclpy.node.Node on which to build this interface
        """
        super().__init__('landmark_finder')
        self._node_inf = node_inf
        apriltag_ns = self._node_inf.get_parameter_or('~apriltag_ns', 'apriltag')

        pkg_path = get_package_share_directory('interbotix_landmark_modules')

        # get parameters
        self.lm_config_filepath = self._node_inf.get_parameter_or(
            '~landmark_config',
            f'{pkg_path}/landmarks/landmarks.yaml'
        )
        self.obs_frame = self._node_inf.get_parameter_or(
            '~obs_frame',
            'camera_color_optical_frame'
        )
        self.fixed_frame = self._node_inf.get_parameter_or(
            '~fixed_frame',
            'landmarks'
        )

        self.valid_tags = None
        self.landmarks = LandmarkCollection(
            node_inf=self._node_inf,
            landmarks={},
            obs_frame=self.obs_frame,
            fixed_frame=self.fixed_frame,
            ros_on=True)

        self._load_landmarks()
        self.apriltag = AprilTag(
            apriltag_ns=apriltag_ns,
            node_inf=self._node_inf
        )
        self.apriltag.set_valid_tags(self.valid_tags)

        # wait a bit for other nodes to launch
        time.sleep(5.0)

        self.timer = self._node_inf.create_timer(
            timer_period_sec=2.0,
            callback=self.timer_callback
        )
        self._node_inf.get_logger().info('Initialized LandmarkFinder!')

        # self.node_inf.on_shutdown(self._save_landmarks)  # TODO(lsinterbotix)

    def _load_landmarks(self) -> None:
        """Load collection of landmarks from the specified filepath."""
        # load landmarks from filepath
        if self.landmarks.load(self.lm_config_filepath):
            # set tags seen in landmark file
            self.valid_tags = self.landmarks.get_valid_tags()
            self._node_inf.get_logger().info(f'Loaded landmarks from {self.lm_config_filepath}.')

    def _save_landmarks(self, tags: Union[List[int], None] = None) -> None:
        """
        Save specified tag IDs of landmarks.

        :param tags: (optional) tag IDs of the landmarks to save. Defaults to `None`
        """
        self.landmarks.save(filepath=self.lm_config_filepath, ids=tags)

    def timer_callback(self) -> None:
        """Publish transformed TF from observation to fixed frame if a tag is seen."""
        # get pose with respect to camera frame
        poses_wrt_cam, tag_ids = self.apriltag.find_pose_id()
        # loop through detected tags, if none, skip
        num_tags = len(tag_ids)
        if num_tags > 0:
            for i in range(num_tags):
                # if we get a valid pose (if given quaternion is valid)...
                if quaternion_is_valid(poses_wrt_cam[i].orientation):
                    # transform the pose to the new frame
                    self.landmarks.get_landmark(tag_ids[i]).tf_wrt_cam = poses_wrt_cam[i]
                    self.landmarks.get_landmark(tag_ids[i]).update_tf(
                        parent_old=self.obs_frame,
                        parent_new=self.fixed_frame
                    )
            # publish
            self.landmarks.pub_tfs(tag_ids)
            time.sleep(1.0)  # wait to make sure tf is published
            self.landmarks.pub_markers(tag_ids)


def main(args=None):
    rclpy.init(args=args)
    try:
        landmark_finder = LandmarkFinder()
        rclpy.spin(landmark_finder)
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
