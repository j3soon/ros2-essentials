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

from typing import List, Tuple

# Tuple of valid Interbotix X-Series arm models
_XSARM_MODELS = (
    'px100',
    'px150',
    'rx150',
    'rx200',
    'wx200',
    'wx250',
    'wx250s',
    'vx250',
    'vx300',
    'vx300s',
    'mobile_px100',
    'mobile_wx200',
    'mobile_wx250s',
    'aloha_wx250s',
    'aloha_vx300s',
)

# Tuple of valid Interbotix LoCoBot models
_XSLOCOBOT_MODELS = (
    'locobot_base',
    'locobot_px100',
    'locobot_wx200',
    'locobot_wx250s',
)

# Tuple of valid Interbotix Turret models
_XSTURRET_MODELS = (
    'pxxls_cam',
    'pxxls',
    'vxxmd',
    'vxxms',
    'wxxmd',
    'wxxms',
)


def get_interbotix_xsarm_models() -> Tuple[str]:
    """Get the tuple of valid Interbotix X-Series arm models."""
    return _XSARM_MODELS


def get_interbotix_xslocobot_models() -> Tuple[str]:
    """Get the tuple of valid Interbotix LoCoBot models."""
    return _XSLOCOBOT_MODELS


def get_interbotix_xsturret_models() -> Tuple[str]:
    """Get the tuple of valid Interbotix Turret models."""
    return _XSTURRET_MODELS


def get_interbotix_xsarm_joints(robot_model: str) -> List[str]:
    """
    Return a list of joints in the robot_model.

    :param robot_model: The robot model to get the joints of
    :return: A list of joint names of the given robot model
    :raises: KeyError if the robot model is not valid
    """
    if robot_model in ('mobile_px100', 'px100'):
        return ['waist', 'shoulder', 'elbow', 'wrist_angle', 'left_finger']
    elif robot_model in ('px150', 'rx150', 'rx200', 'wx200', 'wx250', 'vx250', 'vx300'):
        return ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'left_finger']
    elif robot_model in ('mobile_wx250s', 'wx250s', 'vx300s', 'aloha_wx250s', 'aloha_vx300s'):
        return [
            'waist', 'shoulder', 'elbow', 'forearm_roll',
            'wrist_angle', 'wrist_rotate', 'left_finger'
        ]
    else:
        raise KeyError(f'{robot_model} is not a valid robot model.')
