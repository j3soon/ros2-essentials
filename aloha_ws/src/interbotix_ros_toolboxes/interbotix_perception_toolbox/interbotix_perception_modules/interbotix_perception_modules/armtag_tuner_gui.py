#!/usr/bin/env python3

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

import argparse
import os
import signal
import sys
from typing import Dict

from ament_index_python import get_package_share_directory, PackageNotFoundError
from interbotix_common_modules.common_robot.robot import create_interbotix_global_node
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QApplication, QLineEdit, QWidget
from rclpy.utilities import remove_ros_args


class NameMapElement:

    display: QLineEdit
    get_func: property

    def __init__(self, *, display=None, get_func=None):
        self.display = display
        self.get_func = get_func


class GUI(QWidget):
    """Placeholder QWidget."""


class ArmTagTunerGui(QWidget):
    """GUI to snap AR tag pose on the arm and calculate the 'ref to arm base_link' transform."""

    def __init__(self, ros_args, node_inf, args):
        """Construct ArmTagTunerGUI object."""
        super(ArmTagTunerGui, self).__init__()
        self.armtag_inf = InterbotixArmTagInterface(
            armtag_ns=ros_args.armtag_ns,
            apriltag_ns=ros_args.apriltag_ns,
            node_inf=node_inf,
            args=args,
        )
        try:
            self.pkg_share_path = get_package_share_directory('interbotix_perception_modules')
        except PackageNotFoundError as err:
            self.armtag_inf.apriltag_inf.get_logger().error((
                "Could not find the package 'interbotix_perception_modules'.\n"
                "Error was '%s'." % err
            ))
            exit(1)
        loadUi(
            os.path.join(
                self.pkg_share_path,
                'ui',
                'armtagtunergui.ui'
            ),
            self,
            {'GUI': GUI}
        )

        self.armtag_inf.apriltag_inf.node_inf.declare_parameter(
            'position_only',
            'false'
        )

        self.position_only = self.armtag_inf.apriltag_inf.node_inf.get_parameter(
            'position_only').get_parameter_value().bool_value

        self.name_map: Dict[str, NameMapElement] = {}
        self.setWindowIcon(
            QIcon(
                os.path.join(
                    self.pkg_share_path,
                    'gui',
                    'icon',
                    'Interbotix_Circle.png'
                )
            )
        )
        self.create_snap_block()
        self.create_display_block()
        self.show()
        self.armtag_inf.apriltag_inf.node_inf.get_logger().info('ArmTag Tuner GUI is up!')

    def create_snap_block(self):
        """Create the Snap Block containing the Snap Pose button and Num Samples box."""
        self.button_snappose.clicked.connect(self.snap_pose)

    def create_display_block(self):
        """Create a display to show the 'ref to arm base_link' transform."""
        self.label_description.setText((
            f"Snapped pose represents the transform from\n '{self.armtag_inf.parent_frame}' "
            f"to '{self.armtag_inf.child_frame}'."
        ))
        self.name_map['X [m]'] = NameMapElement(
            display=self.lineedit_x,
            get_func=self.armtag_inf.get_x)
        self.name_map['Y [m]'] = NameMapElement(
            display=self.lineedit_y,
            get_func=self.armtag_inf.get_y)
        self.name_map['Z [m]'] = NameMapElement(
            display=self.lineedit_z,
            get_func=self.armtag_inf.get_z)
        self.name_map['Roll [rad]'] = NameMapElement(
            display=self.lineedit_roll,
            get_func=self.armtag_inf.get_roll)
        self.name_map['Pitch [rad]'] = NameMapElement(
            display=self.lineedit_pitch,
            get_func=self.armtag_inf.get_pitch)
        self.name_map['Yaw [rad]'] = NameMapElement(
            display=self.lineedit_yaw,
            get_func=self.armtag_inf.get_yaw)

    def snap_pose(self):
        """Handle 'Snap Pose' button event (snaps the AR tag pose)."""
        if self.armtag_inf.find_ref_to_arm_base_transform(
            num_samples=self.spinbox_numsamples.value(),
            position_only=self.position_only
        ):
            for element in self.name_map.values():
                element.display.setText(str(round(element.get_func(), 3)))
            self.label_status.setText('Successfully found and published transform.')
        else:
            self.label_status.setText((
                'Was not successful in finding and publishing transform.\n'
                'Can the camera see the tag?'
            ))


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--armtag_ns', default='armtag')
    p.add_argument('--apriltag_ns', default='apriltag')
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    global_node = create_interbotix_global_node()

    app = QApplication(sys.argv)
    gui = ArmTagTunerGui(ros_args, global_node, args)  # noqa: F841

    # Only kill the program at node shutdown
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
