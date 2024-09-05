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
from typing import Callable, Union

from ament_index_python import get_package_share_directory, PackageNotFoundError
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    InterbotixRobotNode,
    robot_startup,
)
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QSlider,
    QSpinBox,
    QWidget,
)
from rclpy.utilities import remove_ros_args


app = None


class GUI(QWidget):
    """Placeholder QWidget."""


class PointCloudTunerGui(QWidget):
    """GUI to tune the PointCloud Filtering parameters."""

    def __init__(self, ros_args, node: InterbotixRobotNode, args=None):
        super(PointCloudTunerGui, self).__init__()
        self.pc_inf = InterbotixPointCloudInterface(
            filter_ns=ros_args.filter_ns,
            node_inf=node,
            args=args,
        )
        try:
            self.pkg_share_path = get_package_share_directory('interbotix_perception_modules')
        except PackageNotFoundError as err:
            self.pc_inf.get_logger().error((
                "Could not find the package 'interbotix_perception_modules'.\n"
                f"Error was '{err}'."))
            exit(1)
        loadUi(
            os.path.join(
                self.pkg_share_path,
                'ui',
                'pointcloudtunergui.ui'
            ),
            self,
            {'GUI': GUI}
        )
        self.name_map = {}
        self.filepath = self.pc_inf.get_filepath()
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
        self.create_crop_box_block()
        self.create_voxel_grid_block()
        self.create_seg_block()
        self.create_ror_block()
        self.create_cluster_block()
        self.create_button_block()
        self.show()
        self.pc_inf.node_inf.loginfo('PointCloud Tuner GUI is up!')

    def create_crop_box_block(self):
        """Create the Crop Box Filter Block."""
        self.create_sub_component(
            name='X min [m]',
            set_func=self.pc_inf.set_x_filter_min,
            get_func=self.pc_inf.get_x_filter_min,
            min_val=-0.5,
            max_val=0.5,
            display=self.doublespinbox_cropbox_xmin,
            slider=self.hslider_cropbox_xmin,
            precision=2,
        )
        self.create_sub_component(
            name='X max [m]',
            set_func=self.pc_inf.set_x_filter_max,
            get_func=self.pc_inf.get_x_filter_max,
            min_val=-0.5,
            max_val=0.5,
            display=self.doublespinbox_cropbox_xmax,
            slider=self.hslider_cropbox_xmax,
            precision=2,
        )
        self.create_sub_component(
            name='Y min [m]',
            set_func=self.pc_inf.set_y_filter_min,
            get_func=self.pc_inf.get_y_filter_min,
            min_val=-0.5,
            max_val=0.5,
            display=self.doublespinbox_cropbox_ymin,
            slider=self.hslider_cropbox_ymin,
            precision=2,
        )
        self.create_sub_component(
            name='Y max [m]',
            set_func=self.pc_inf.set_y_filter_max,
            get_func=self.pc_inf.get_y_filter_max,
            min_val=-0.5,
            max_val=0.5,
            display=self.doublespinbox_cropbox_ymax,
            slider=self.hslider_cropbox_ymax,
            precision=2,
        )
        self.create_sub_component(
            name='Z min [m]',
            set_func=self.pc_inf.set_z_filter_min,
            get_func=self.pc_inf.get_z_filter_min,
            min_val=0.2,
            max_val=1.5,
            display=self.doublespinbox_cropbox_zmin,
            slider=self.hslider_cropbox_zmin,
            precision=2,
        )
        self.create_sub_component(
            name='Z max [m]',
            set_func=self.pc_inf.set_z_filter_max,
            get_func=self.pc_inf.get_z_filter_max,
            min_val=0.2,
            max_val=1.5,
            display=self.doublespinbox_cropbox_zmax,
            slider=self.hslider_cropbox_zmax,
            precision=2,
        )

    def create_voxel_grid_block(self):
        """Create the Voxel Filter Block."""
        self.create_sub_component(
            name='Leaf Size [m]',
            set_func=self.pc_inf.set_voxel_leaf_size,
            get_func=self.pc_inf.get_voxel_leaf_size,
            min_val=0.001,
            max_val=0.01,
            display=self.doublespinbox_voxelgrid_leafsize,
            slider=self.hslider_voxelgrid_leafsize,
            precision=3,
        )

    def create_seg_block(self):
        """Create the Plane Segmentation Block."""
        self.create_sub_component(
            name='Dist. Thresh [m]',
            set_func=self.pc_inf.set_plane_dist_thresh,
            get_func=self.pc_inf.get_plane_dist_thresh,
            min_val=0.001,
            max_val=0.05,
            display=self.doublespinbox_seg_thresh,
            slider=self.hslider_seg_thresh,
            precision=3,
        )
        self.create_sub_component(
            name='Max Iterations',
            set_func=self.pc_inf.set_plane_max_iter,
            get_func=self.pc_inf.get_plane_max_iter,
            min_val=25,
            max_val=1000,
            display=self.spinbox_seg_iter,
            slider=self.hslider_seg_iter,
            precision=0,
        )

    def create_ror_block(self):
        """Create the Radius Outlier Removal Block."""
        self.create_sub_component(
            name='Min Neighbors',
            set_func=self.pc_inf.set_ror_min_neighbors,
            get_func=self.pc_inf.get_ror_min_neighbors,
            min_val=1,
            max_val=20,
            display=self.spinbox_outlier_minneighbors,
            slider=self.hslider_outlier_minneighbors,
            precision=0,
        )
        self.create_sub_component(
            name='Radius Search [m]',
            set_func=self.pc_inf.set_ror_radius_search,
            get_func=self.pc_inf.get_ror_radius_search,
            min_val=0.005,
            max_val=0.05,
            display=self.doublespinbox_outlier_radius,
            slider=self.hslider_outlier_radius,
            precision=3,
        )

    def create_cluster_block(self):
        """Create the Cluster Filter Block."""
        self.create_sub_component(
            name='Min Size',
            set_func=self.pc_inf.set_cluster_min_size,
            get_func=self.pc_inf.get_cluster_min_size,
            min_val=25,
            max_val=1000,
            display=self.spinbox_cluster_size_min,
            slider=self.hslider_cluster_size_min,
            precision=0,
        )
        self.create_sub_component(
            name='Max Size',
            set_func=self.pc_inf.set_cluster_max_size,
            get_func=self.pc_inf.get_cluster_max_size,
            min_val=25,
            max_val=1000,
            display=self.spinbox_cluster_size_max,
            slider=self.hslider_cluster_size_max,
            precision=0,
        )
        self.create_sub_component(
            name='Tolerance [m]',
            set_func=self.pc_inf.set_cluster_tol,
            get_func=self.pc_inf.get_cluster_tol,
            min_val=0.01,
            max_val=0.1,
            display=self.doublespinbox_cluster_tol,
            slider=self.hslider_cluster_tol,
            precision=3,
        )

    def create_button_block(self):
        """Create GUI subsection for Load, Save, and Reset Configs buttons."""
        self.button_config_reset.clicked.connect(self.reset_configs)
        self.button_config_load.clicked.connect(self.load_configs)
        self.button_config_save.clicked.connect(self.save_configs)

    def create_sub_component(
        self,
        name: str,
        set_func: Callable,
        get_func: Callable,
        min_val: Union[float, int],
        max_val: Union[float, int],
        display: Union[QDoubleSpinBox, QSpinBox],
        slider: QSlider,
        precision: int,
    ):
        """
        Configure GUI blocks.

        :param name: name of the Block
        :param set_func: function to set the value of a specific param
        :param get_func: function to get the value of a specifc param
        :param min_val: minimum value that the param can have
        :param max_val: maximum value that the param can have
        :param display: Ref to SpinBox or SpinBoxDouble Widget for this param
        :param slider: Ref to Slider Widget for this param
        :param precision: decimal precision of the param in 10**-precision
        """
        # Configure slider
        slider_range = int(round(abs(max_val - min_val)/10**-precision))
        slider_pctvalue = (get_func() - min_val) / float(max_val - min_val)
        slider.setRange(0, slider_range)
        slider.setValue(int(round(slider_pctvalue * slider_range)))

        # Configure display
        display.setValue(get_func())
        display.setRange(min_val, max_val)
        display.setSingleStep(10**-precision)
        if isinstance(display, QDoubleSpinBox):
            display.setDecimals(precision)

        # Configure signals
        display.valueChanged.connect(lambda: self.update_slider_bar(name))
        slider.valueChanged.connect(lambda: self.update_display(name))

        # global dictionary to store and retrieve values
        self.name_map[name] = {
            'display': display,
            'slider': slider,
            'min_val': min_val,
            'max_val': max_val,
            'set_func': set_func,
            'get_func': get_func,
            'range': slider_range,
            'precision': precision
        }

        self.update_display(name)

# Event Handlers

    def update_slider_bar(self, name):
        """
        Handle when a display is changed.

        :param name: name of the sub-component where the value was changed
        :details: Updates the slider position to reflect that shown in the display
        """
        info = self.name_map[name]
        value = float(info['display'].text())
        if (value < info['min_val']):
            value = info['min_val']
            info['display'].setValue(value)
        elif (value > info['max_val']):
            value = info['max_val']
            info['display'].setValue(value)
        pctvalue = (value - info['min_val']) / float(info['max_val'] - info['min_val'])
        info['slider'].setValue(int(round(pctvalue * info['range'])))

    def update_display(self, name):
        """
        Handle when a slider is changed.

        :param name: name of the sub-component where the value was changed
        :details: Updates the display to reflect the position dictated by the slider
        """
        info = self.name_map[name]
        slider_value = info['slider'].value()
        pctvalue = slider_value / float(info['range'])
        value = pctvalue * (info['max_val'] - info['min_val']) + info['min_val']
        num = ('%.' + str(info['precision']) + 'f') % value
        # check if value is an integer or float
        if info['precision'] == 0:
            info['display'].setValue(int(num))
        else:
            info['display'].setValue(float(num))
        info['set_func'](value)

    def reset_configs(self):
        """
        Handle the 'Reset Configs' button event.

        :details: Resets the displays and sliders to the values as defined in the loaded YAML file
        """
        self.pc_inf.load_params(self.filepath)
        for info in self.name_map.values():
            pctvalue = (
                (info['get_func']() - info['min_val']) / float(info['max_val'] - info['min_val'])
            )
            info['slider'].setValue(int(round(pctvalue * info['range'])))

    def load_configs(self):
        """
        Handle the 'Load Configs' button event.

        :details: Opens up a dialogue box where the user can specify the YAML file to load
        """
        fname = QFileDialog.getOpenFileName(
            self,
            'Open file',
            self.filepath,
            'YAML files (*.yaml)'
        )
        if (fname[0] == ''):
            return
        self.filepath = fname[0]
        self.pc_inf.load_params(self.filepath)
        for info in self.name_map.values():
            pctvalue = (
                (info['get_func']() - info['min_val']) / float(info['max_val'] - info['min_val'])
            )
            info['slider'].setValue(int(round(pctvalue * info['range'])))

    def save_configs(self):
        """
        Handle 'Save Configs' button event.

        :details: Opens up a dialogue box where the user can specify the YAML file to which to save
            the parameters
        """
        fname = QFileDialog.getSaveFileName(
            self,
            'Save file',
            self.filepath,
            'YAML files (*.yaml)'
        )
        if (fname[0] == ''):
            return
        self.filepath = fname[0]
        self.pc_inf.save_params(self.filepath)


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--filter_ns')
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    global_node = create_interbotix_global_node()

    app = QApplication(sys.argv)
    gui = PointCloudTunerGui(ros_args, global_node, args)  # noqa: F841

    robot_startup(global_node)

    # Only kill the program at node shutdown
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
