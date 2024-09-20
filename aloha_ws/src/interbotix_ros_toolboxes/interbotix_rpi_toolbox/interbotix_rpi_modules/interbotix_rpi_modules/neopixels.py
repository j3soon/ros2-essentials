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

"""Contains the `InterbotixRpiPixelInterface` class that interfaces with the rpi_pixels node."""

import time

from interbotix_rpi_msgs.msg import PixelCommands
from rclpy.node import Node


class InterbotixRpiPixelInterface:
    """Class that interfaces with the rpi_pixels node."""

    def __init__(
        self,
        robot_name: str,
        core: Node,
    ) -> None:
        """
        Construct the InterbotixRpiPixelInterface object.

        :param robot_name: The name of the robot
        :param core: Reference to a ROS2 node
        """
        self.core = core
        self.pub_pixels = self.core.create_publisher(
            PixelCommands,
            f'/{robot_name}/commands/pixels',
            5,
        )
        time.sleep(1)

    def set_color(
        self,
        pixel: int = 0,
        color: int = 0x000000,
        set_all_leds: bool = False
    ) -> None:
        """
        Set the color of an LED, or all LEDs.

        :param pixel: (optional) The pixel number to set the color of; defaults to 0
        :param color: (optional) Hex value of the color to change to; defaults to 0x000000 (black)
        :param set_all_leds: (optional) `True` to set all LEDs to the specified color, `False` to
            set only the LED specified in the `pixel` param; defaults to `False`
        """
        msg = PixelCommands()
        msg.cmd_type = 'color'
        msg.set_all_leds = set_all_leds
        msg.pixel = pixel
        msg.color = color
        self.pub_pixels.publish(msg)

    def set_brightness(self, brightness: int = 0) -> None:
        """
        Set the brighness of all LEDs.

        :param brightness: (optional) The brightness to set all LEDs to; defaults to 0
        """
        msg = PixelCommands()
        msg.cmd_type = 'brightness'
        msg.brightness = brightness
        self.pub_pixels.publish(msg)

    def pulse(self, iterations: int = 5, period: int = 10) -> None:
        """
        Pulse all LEDs some number of times with a specified period per pulse.

        Pulsing the LEDs will slowly fade in and slowly fade out some number of times.

        :param iterations: (optional) Number of times to pulse the LEDs; defaults to 5
        :param period: (optional) Period of each pulse in seconds; defaults to 10
        """
        msg = PixelCommands()
        msg.cmd_type = 'pulse'
        msg.period = period
        msg.iterations = iterations
        self.pub_pixels.publish(msg)

    def blink(
        self,
        pixel: int = 0,
        set_all_leds: bool = False,
        period: int = 500,
        iterations: int = 3
    ) -> None:
        """
        Blink one or all LEDs some number of times with a specified period per blink.

        :param pixel: (optional) The pixel number to blink; defaults to 0
        :param set_all_leds: (optional) `True` to blink all LEDs, `False` to blink only the LED
            specified in the `pixel` param; defaults to `False`
        :param period: (optional) Time in milliseconds for each blink; defaults to 500
        :param iterations: (optional) Number of times to blink the LEDs; defaults to 3
        """
        msg = PixelCommands()
        msg.cmd_type = 'blink'
        msg.set_all_leds = set_all_leds
        msg.pixel = pixel
        msg.period = period
        msg.iterations = iterations
        self.pub_pixels.publish(msg)
