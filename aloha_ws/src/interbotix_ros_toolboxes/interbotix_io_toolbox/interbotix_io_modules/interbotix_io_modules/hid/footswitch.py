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

"""Contains a class used to get feedback from and process callbacks related to a footswitch HID."""

from dataclasses import dataclass
from enum import (
    auto,
    Enum,
)
from typing import Callable

from interbotix_common_modules.common_robot.robot import InterbotixRobotNode
from interbotix_footswitch_msgs.msg import FootswitchState


class FootswitchTrigger(Enum):
    """Enum defining when callbacks should be triggered based on the change in footswitch state."""

    ON_PRESS = auto()
    """Callbacks are triggered when the pedal is pressed"""

    ON_RELEASE = auto()
    """Callbacks are triggered when the pedal is released"""

    ON_CHANGE = auto()
    """Callbacks are triggered when the pedal is pressed or released"""


@dataclass
class IndividualFSConfig:
    """Configuration for individual pedals on a footswitch."""

    trigger: FootswitchTrigger = FootswitchTrigger.ON_PRESS
    """When callbacks should be triggered"""

    callback: Callable = lambda: None
    """The callback that should be triggered. Defaults to a null function"""


@dataclass
class InterbotixFootswitchConfig:
    """Configuration for a footswitch mapping pedals and transitions to callbacks."""

    config: tuple[IndividualFSConfig, IndividualFSConfig, IndividualFSConfig]
    """Per-pedal configuration. Tuple index corresponds to the index of the pedal"""


class InterbotixFootswitch:

    def __init__(
        self,
        node: InterbotixRobotNode,
        config: InterbotixFootswitchConfig,
        state_topic: str = 'state',
    ) -> None:
        """
        Construct the Interbotix Footswitch module.

        :param node: The InterbotixRobotNode to base this class's ROS components on
        :param config: The configuration for the footswitch module
        :param state_topic: The FootswitchState topic this module should subscribe to, defaults to
            'state'
        """
        self.node = node
        self.state = FootswitchState().state
        self.config = config
        self.state_sub = self.node.create_subscription(
            FootswitchState,
            state_topic,
            self.callback_process_state,
            1,
        )
        self.node.get_logger().info('Initialized InterbotixFootswitch!')

    def callback_process_state(self, msg: FootswitchState) -> None:
        """
        Process incoming FootswitchState messages.

        :param msg: The incoming FootswitchState message
        """
        self.state_prev = self.state
        self.state = msg.state
        for idx, state in enumerate(self.state):
            match self.config.config[idx].trigger:
                case FootswitchTrigger.ON_PRESS:
                    if not self.state_prev[idx] and state:
                        self.config.config[idx].callback()
                case FootswitchTrigger.ON_RELEASE:
                    if self.state_prev[idx] and not state:
                        self.config.config[idx].callback()
                    pass
                case FootswitchTrigger.ON_CHANGE:
                    if self.state_prev[idx] != state:
                        self.config.config[idx].callback()
                case _:
                    pass
