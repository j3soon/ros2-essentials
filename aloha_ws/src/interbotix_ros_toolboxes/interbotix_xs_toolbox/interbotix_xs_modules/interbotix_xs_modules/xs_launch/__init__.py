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

"""
Declutter X-Series Python launch files.

The `xs_launch` module helps declutter Interbotix X-Series Python launch files by providing useful
helper functions and classes.
"""

from .xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    construct_interbotix_xslocobot_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
    declare_interbotix_xslocobot_robot_description_launch_arguments,
    declare_interbotix_xsturret_robot_description_launch_arguments,
    determine_use_sim_time_param,
)

__all__ = [
    'construct_interbotix_xsarm_semantic_robot_description_command',
    'construct_interbotix_xslocobot_semantic_robot_description_command',
    'declare_interbotix_xsarm_robot_description_launch_arguments',
    'declare_interbotix_xslocobot_robot_description_launch_arguments',
    'declare_interbotix_xsturret_robot_description_launch_arguments',
    'determine_use_sim_time_param',
]
