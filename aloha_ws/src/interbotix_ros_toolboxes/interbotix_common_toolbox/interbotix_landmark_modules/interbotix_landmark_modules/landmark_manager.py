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

from typing import Any

from ament_index_python import get_package_share_directory
from interbotix_landmark_modules.landmark import LandmarkCollection
import rclpy
from rclpy.node import Node

CURSOR = '  >  '


class LandmarkManagerCLI(Node):
    """Command Line Interface used to define and enable user-specific landmarks."""

    def __init__(self):
        """Construct the LandmarkManagerCLI object."""
        super().__init__('landmark_manager')
        self.get_logger().info(
            '\n\nThis CLI application will be used to define and enable user-specified landmarks.'
        )
        lm_path = get_package_share_directory('interbotix_landmark_modules')

        # load params
        self.filepath = self.get_parameter_or(
            '~landmark_config',
            f'{lm_path}/landmarks/landmarks.yaml')
        self.fixed_frame = self.get_parameter_or(
            '~fixed_frame',
            'landmarks')
        self.obs_frame = self.get_parameter_or(
            '~obs_frame',
            'camera_color_optical_frame')
        param_tags = self.get_parameter_or(
            '~standalone_tags',
            [
                {'id':   5, 'size': 0.02},
                {'id':  413, 'size': 0.02},
                {'id':  820, 'size': 0.02},
                {'id':  875, 'size': 0.02},
                {'id': 1050, 'size': 0.02}
            ]
        )
        self.valid_tags = [tag['id'] for tag in param_tags]

        self.landmarks = LandmarkCollection(
            node_inf=self,
            landmarks={},
            fixed_frame=self.fixed_frame,
            observation_frame=self.obs_frame
        )

        self.landmarks.load(self.filepath)
        self.add_landmarks()
        self.save_landmarks()

    def add_landmarks(self):
        """Walk the user through adding landmarks to collection."""
        while True:
            self.get_logger().info((
                f'\nSelect landmark id you wish to create. Options: \n\t{self._print_tags()}'
            ))
            while True:
                id_ = self._get_input(int)
                if id_ in self.valid_tags:
                    break
                else:
                    self.get_logger().warn(
                        f'\nPlease choose from the list of valid tags: {self.valid_tags}'
                    )

            self.get_logger().info('\nLandmark label (all lowercase).')
            label = self._get_input(str).lower()
            self.landmarks.add_landmark(label=label, id_num=id_)

            self.get_logger().info('\nIs the tag mounted? Options: [y/n]')
            mounted_yn = self._get_input(str).lower()
            if (mounted_yn == 'y') or (mounted_yn == 'yes'):
                mounted = True
                self.get_logger().info('\nOffset in meters: ')
                mounted_offset = self._get_input(float)
            else:
                mounted = False
                mounted_offset = 0.0
            self.landmarks.get_landmark(id_).set_mounted(mounted)
            self.landmarks.get_landmark(id_).set_mounted_offset(mounted_offset)

            self.get_logger().info('\nEnter another landmark? Options: [y/n]')
            more = self._get_input(str).lower()
            if not ((more == 'y') or (more == 'yes')):
                self.get_logger().info('\nDone adding landmarks. Saving and closing...\n\n')
                break

    def _get_input(self, type_: Any) -> Any:
        """
        Get robust user input by repeating prompt until it can properly transform the input.

        :param type_: desired input type
        :return: properly formatted user input
        """
        while True:
            try:
                input_ = type_(input(CURSOR))
                return input_
            except ValueError:
                self.get_logger().warn(f'\nPlease enter a value of {type_}')

    def save_landmarks(self) -> None:
        """Call the LandmarkCollection.save() function."""
        self.landmarks.save(self.filepath)

    def _print_tags(self) -> str:
        """
        Format the list of tags to highlight those with set landmarks.

        :return: string with formatted list of tags
        """
        tags_in_collection = self.landmarks.get_valid_tags()
        tag_list = ''
        for tag in self.valid_tags:
            tag_list += (f'({tag}) ' if tag in tags_in_collection else f'{tag} ')
        return '[ ' + tag_list + ']'


def main(args=None):
    rclpy.init(args=args)
    try:
        LandmarkManagerCLI()
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
