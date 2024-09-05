# interbotix_common_modules

## Overview
This package contains helper Python modules that can be applied in many robotic platforms and are not hardware specific. They are used to create other modules in the various toolboxes.

## Structure
Below is a list and short description of each helper module. Over time, this list will grow to include others. 

- [angle_manipulation](src/interbotix_xs_modules/angle_manipulation.py) - small library of functions to convert Euler angles to rotation matrices and visa versa.

## Usage
While the modules in this package are mainly meant to be used in the other toolboxes, they can also be imported into your own Python scripts. To import, type `from interbotix_common_modules import <module>`.
