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

"""
Modern Robotics Descriptions for all Interbotix X-Series Arms.

This module contains properties required by the Modern Robotics library to perform
kinematic calculations such as the Joint Screw Axes and Home Configuration for each
X-Series robot.

Note that the end-effector is positioned at '<robot_name>/ee_gripper_link' and that the
Space frame is positioned at '<robot_name>/base_link'.

To calculate your own MR Description, check out the kinematics_from_description package:
    https://github.com/Interbotix/kinematics_from_description
"""

from abc import ABC

import numpy as np


class ModernRoboticsDescription(ABC):
    """
    Abstract base class for other MR Description classes.

    Derived classes should override the Slist and M member variables.
    """

    Slist: np.ndarray = None
    """
    Joint screw axes in the space frame when the manipulator is at the home position, in the format
    of a matrix with axes as the columns
    """

    M: np.ndarray = None
    """The home configuration (position and orientation) of the end-effector"""


class px100(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,    0.0, 0.0],
                      [0.0, 1.0, 0.0, -0.0931, 0.0, 0.0],
                      [0.0, 1.0, 0.0, -0.1931, 0.0, 0.035],
                      [0.0, 1.0, 0.0, -0.1931, 0.0, 0.135]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.248575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.1931],
                  [0.0, 0.0, 0.0, 1.0]])


class px150(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.10457, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.25457, 0.0,     0.05],
                      [0.0, 1.0, 0.0, -0.25457, 0.0,     0.2],
                      [1.0, 0.0, 0.0,  0.0,     0.25457, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.358575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.25457],
                  [0.0, 0.0, 0.0, 1.0]])


class rx150(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.10457, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.25457, 0.0,     0.05],
                      [0.0, 1.0, 0.0, -0.25457, 0.0,     0.2],
                      [1.0, 0.0, 0.0,  0.0,     0.25457, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.358575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.25457],
                  [0.0, 0.0, 0.0, 1.0]])


class rx200(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.10457, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.30457, 0.0,     0.05],
                      [0.0, 1.0, 0.0, -0.30457, 0.0,     0.25],
                      [1.0, 0.0, 0.0,  0.0,     0.30457, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.408575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.30457],
                  [0.0, 0.0, 0.0, 1.0]])


class vx250(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.37705, 0.0,     0.06],
                      [0.0, 1.0, 0.0, -0.37705, 0.0,     0.31],
                      [1.0, 0.0, 0.0,  0.0,     0.37705, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.468575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.37705],
                  [0.0, 0.0, 0.0, 1.0]])


class vx300(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                      [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                      [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.536494],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.42705],
                  [0.0, 0.0, 0.0, 1.0]])


class vx300s(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                      [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                      [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                      [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.536494],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.42705],
                  [0.0, 0.0, 0.0, 1.0]])


class wx200(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.11065, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.31065, 0.0,     0.05],
                      [0.0, 1.0, 0.0, -0.31065, 0.0,     0.25],
                      [1.0, 0.0, 0.0,  0.0,     0.31065, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.408575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.31065],
                  [0.0, 0.0, 0.0, 1.0]])


class wx250(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.11065, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.36065, 0.0,     0.04975],
                      [0.0, 1.0, 0.0, -0.36065, 0.0,     0.29975],
                      [1.0, 0.0, 0.0,  0.0,     0.36065, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.458325],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.36065],
                  [0.0, 0.0, 0.0, 1.0]])


class wx250s(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.11065, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.36065, 0.0,     0.04975],
                      [1.0, 0.0, 0.0,  0.0,     0.36065, 0.0],
                      [0.0, 1.0, 0.0, -0.36065, 0.0,     0.29975],
                      [1.0, 0.0, 0.0,  0.0,     0.36065, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.458325],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.36065],
                  [0.0, 0.0, 0.0, 1.0]])


class mobile_px100(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0, 0.0],
                      [0.0, 1.0, 0.0, -0.08518, 0.0, 0.0],
                      [0.0, 1.0, 0.0, -0.18518, 0.0, 0.035],
                      [0.0, 1.0, 0.0, -0.18518, 0.0, 0.135]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.248575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.18518],
                  [0.0, 0.0, 0.0, 1.0]])


class mobile_wx200(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,      0.0,      0.0],
                      [0.0, 1.0, 0.0, -0.104825, 0.0,      0.0],
                      [0.0, 1.0, 0.0, -0.304825, 0.0,      0.05],
                      [0.0, 1.0, 0.0, -0.304825, 0.0,      0.25],
                      [1.0, 0.0, 0.0,  0.0,      0.304825, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.408575],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.304825],
                  [0.0, 0.0, 0.0, 1.0]])


class mobile_wx250s(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,      0.0,      0.0],
                      [0.0, 1.0, 0.0, -0.104825, 0.0,      0.0],
                      [0.0, 1.0, 0.0, -0.354825, 0.0,      0.04975],
                      [1.0, 0.0, 0.0,  0.0,      0.354825, 0.0],
                      [0.0, 1.0, 0.0, -0.354825, 0.0,      0.29975],
                      [1.0, 0.0, 0.0,  0.0,      0.354825, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.458325],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.354825],
                  [0.0, 0.0, 0.0, 1.0]])


class aloha_wx250s(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,      0.0,      0.0],
                      [0.0, 1.0, 0.0, -0.104825, 0.0,      0.0],
                      [0.0, 1.0, 0.0, -0.354825, 0.0,      0.04975],
                      [1.0, 0.0, 0.0,  0.0,      0.354825, 0.0],
                      [0.0, 1.0, 0.0, -0.354825, 0.0,      0.29975],
                      [1.0, 0.0, 0.0,  0.0,      0.354825, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.458325],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.354825],
                  [0.0, 0.0, 0.0, 1.0]])


class aloha_vx300s(ModernRoboticsDescription):
    Slist = np.array([[0.0, 0.0, 1.0,  0.0,     0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.12705, 0.0,     0.0],
                      [0.0, 1.0, 0.0, -0.42705, 0.0,     0.05955],
                      [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0],
                      [0.0, 1.0, 0.0, -0.42705, 0.0,     0.35955],
                      [1.0, 0.0, 0.0,  0.0,     0.42705, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.536494],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.42705],
                  [0.0, 0.0, 0.0, 1.0]])
