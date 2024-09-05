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

import os
from typing import List

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
import yaml


class StaticTransformManager(Node):
    """
    Class to manage the publishing of static transforms.

    In ROS, there can only be one static transform publisher per node so any Python
    module that wants to publish a static transform publishes the transform to this node
    via a topic; then this node aggregates and sends all the transforms
    """

    def __init__(self):
        super().__init__('static_trans_pub')
        self.transform_list: List[TransformStamped] = []
        self.br = StaticTransformBroadcaster(self)

        # declare parameters
        self.declare_parameter(
            'load_transforms',
            True)
        self.declare_parameter(
            'save_transforms',
            True)
        self.declare_parameter(
            'transform_filepath',
            os.path.join(
                get_package_share_directory('interbotix_tf_tools'),
                'config',
                'static_transforms.yaml'))

        # get save/load params
        self.load_transforms_param = self.get_parameter(
            'load_transforms').get_parameter_value().bool_value
        self.save_transforms_param = self.get_parameter(
            'save_transforms').get_parameter_value().bool_value

        # if save or load, do file and directory management
        if self.save_transforms_param or self.load_transforms_param:
            self.filepath = self.get_parameter(
                'transform_filepath').get_parameter_value().string_value
            filedir = os.path.dirname(self.filepath)
            self.get_logger().info(f"Transform filepath set to '{self.filepath}'.")

            try:  # check if directory exists, if not make one
                if not os.path.exists(filedir):
                    os.makedirs(filedir)
                    self.get_logger().info(f"Creating directory: '{filedir}'.")
            except (OSError, EnvironmentError) as e:  # if we fail (permissions, etc.)
                self.get_logger().warning(
                    f"Failed to create directory. Transform file will not be saved: '{e}'."
                )

            # load if we need to
            if self.load_transforms_param:
                self.load_transforms()

            # save and set up saving service if we need to
            if self.save_transforms_param:
                self.create_service(Trigger, 'save_transforms', self.trigger_cb)

        self.create_subscription(TransformStamped, 'static_transforms', self.transform_cb, 10)

        self.get_logger().info('Initialized Static Transform Publisher!')

    def transform_cb(self, msg: TransformStamped):
        """
        Receive TransformStamped messages through ROS subscriber callback.

        :param msg: TransformStamped ROS message
        :details: any new static transform is appended to the self.transform_list variable; if the
            transform already exists, the old one is replaced with the new one
        """
        for indx in range(len(self.transform_list)):
            if msg.child_frame_id == self.transform_list[indx].child_frame_id:
                self.transform_list.pop(indx)
                break
        self.transform_list.append(msg)
        self.br.sendTransform(self.transform_list)
        self.get_logger().info((
            'Static Transform Publisher broadcasted TF from '
            f"'{msg.header.frame_id}' to '{msg.child_frame_id}'."
        ))
        if self.save_transforms_param:
            self.save_transforms()

    def load_transforms(self):
        """Load and publish the transforms from config file."""
        if os.path.isfile(self.filepath):
            try:
                with open(self.filepath, 'r') as yamlfile:
                    trans_list = yaml.safe_load(yamlfile)
                self.get_logger().info('Loaded static transforms.')

                self.transform_list = []
                for trans_dict in trans_list:
                    trans = TransformStamped()
                    trans.header.frame_id = trans_dict['frame_id']
                    trans.child_frame_id = trans_dict['child_frame_id']
                    trans.transform.translation.x = trans_dict['x']
                    trans.transform.translation.y = trans_dict['y']
                    trans.transform.translation.z = trans_dict['z']
                    trans.transform.rotation.x = trans_dict['qx']
                    trans.transform.rotation.y = trans_dict['qy']
                    trans.transform.rotation.z = trans_dict['qz']
                    trans.transform.rotation.w = trans_dict['qw']
                    trans.header.stamp = self.get_clock().now().to_msg()
                    self.transform_list.append(trans)
                self.br.sendTransform(self.transform_list)

            except (IOError, OSError):
                self.get_logger().warn(
                    f"Error when opening file '{self.filepath}'. No static transforms loaded."
                )
        else:
            self.get_logger().info(
                'Static transforms file does not exist. No static transforms loaded.'
            )

    def save_transforms(self):
        """
        Save all static transforms to the YAML file specified at 'self.transform_filepath'.

        :details: if no transforms were published to this node, no file is created and no
            transforms are saved
        """
        if self.transform_list == []:
            return

        trans_list = []
        for trans in self.transform_list:
            trans_dict = {}
            trans_dict['frame_id'] = trans.header.frame_id
            trans_dict['child_frame_id'] = trans.child_frame_id
            trans_dict['x'] = float(trans.transform.translation.x)
            trans_dict['y'] = float(trans.transform.translation.y)
            trans_dict['z'] = float(trans.transform.translation.z)
            trans_dict['qx'] = float(trans.transform.rotation.x)
            trans_dict['qy'] = float(trans.transform.rotation.y)
            trans_dict['qz'] = float(trans.transform.rotation.z)
            trans_dict['qw'] = float(trans.transform.rotation.w)
            trans_list.append(trans_dict)

        with open(self.filepath, 'w') as yamlfile:
            yaml.dump(trans_list, yamlfile, default_flow_style=False)

        self.get_logger().info(f"Saved static transforms to: '{self.filepath}'.")

    def trigger_cb(self, req: Trigger.Request, res: Trigger.Response):
        """
        Service callback to save transforms now, instead of on node shutdown.

        :returns: Trigger.Response stating that the TFs were saved
        """
        try:
            self.save_transforms()
            return Trigger.Response(
                success=True,
                message='Static transforms saved successfully.')
        except OSError as e:
            self.get_logger().error(f'Something went wrong when saving static transforms: {e}.')
            return Trigger.Response(
                success=False,
                message='Static transforms NOT saved successfully.')


def main(args=None):
    rclpy.init(args=args)
    try:
        static_transform_manager = StaticTransformManager()
        rclpy.spin(static_transform_manager)
    except KeyboardInterrupt:
        if static_transform_manager.save_transforms_param:
            static_transform_manager.save_transforms()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
