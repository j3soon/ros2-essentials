# interbotix_rpi_modules

## Overview
This package contains ROS nodes and Python modules meant to work on a Raspberry Pi 4B computer running ROS.

## Structure
For these modules to work, the launch file located in any *interbotix_XXXXX_control* package that depends on this package must first be started. Additionally, the desired module must also be imported in your Python script. A brief description of each module is shown below.

- [neopixels](src/interbotix_rpi_modules/neopixels.py) - small library used to control any number of NeoPixel LEDs - including color, brightness, blinking, and pulsing; it contains the InterbotixRpiPixelInterface submodule. To import (for use in your own robot), write `from interbotix_rpi_modules.neopixels import InterbotixRpiPixelInterface` at the top of your Python script.

The ROS nodes in this package are described below.

- **rpi_pixels** - ROS wrapper around the NeoPixel **rpi_ws281x** Python package; it must be run with *sudo* to give the node permissions to access the GPIO pins.

## Usage
Currently, the nodes/modules in this package are being used in our Interbotix X-Series hexapods. Check out the python modules there to get a feel for how to work with them!
