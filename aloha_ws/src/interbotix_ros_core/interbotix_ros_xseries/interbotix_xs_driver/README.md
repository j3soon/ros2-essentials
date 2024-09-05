# Interbotix X-Series Driver

The Interbotix X-Series Driver provides low-level interfaces to easily control the DYNAMIXEL servos on any Interbotix X-Series Robot.

This package can be used as a [standalone C++ library](#using-standalone-c-library), or as [a ROS 2 Ament-style package](#using-ros-2).

#### X-Series Driver Build Status

##### C++ Standalone Build Status
[![build-standalone](https://github.com/Interbotix/interbotix_xs_driver/actions/workflows/standalone.yaml/badge.svg)](https://github.com/Interbotix/interbotix_xs_driver/tree/main)

##### ROS 2-Dependent Build Status
[![build-ros2](https://github.com/Interbotix/interbotix_xs_driver/actions/workflows/ros2.yaml/badge.svg)](https://github.com/Interbotix/interbotix_xs_driver/tree/ros2)

## Installation

This package is known to build on Ubuntu 20.04. No other configuration or environment has been tested and is not supported, though it will probably work on other distributions of Ubuntu.

### Using Standalone C++ Library

#### Required Dependencies

Building the packages requires the following dependencies:

- cmake
  - `sudo apt install -y cmake`
- build-essential
  - `sudo apt install -y build-essential`
- [yaml-cpp-dev](https://launchpad.net/ubuntu/+source/yaml-cpp)
  - `sudo apt install -y yaml-cpp-dev`

#### Building From Source

```sh
git clone --recursive https://github.com/Interbotix/interbotix_xs_driver.git -b main
cd interbotix_xs_driver
mkdir build
cd build
cmake ..
make
sudo make install
```

### Using ROS 2

#### Required Dependencies

Building the packages requires the following dependencies:

- [colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
- [rosdep](http://wiki.ros.org/rosdep#Installing_rosdep)
- The [Interbotix fork](https://github.com/Interbotix/dynamixel-workbench/tree/3ed8229d2382c4d0931b471fbe1c83a4888da6a8) of the [ROBOTIS dynamixel_workbench_toolbox package](https://github.com/ROBOTIS-GIT/dynamixel-workbench)

#### Building

```sh
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/Interbotix/interbotix_xs_driver.git
git clone https://github.com/Interbotix/dynamixel-workbench.git -b ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

The X-Series Driver is compiled as a library and can be used in any C++ project by simply including its headers and linking against it.

```c++
#include "interbotix_xs_driver/xs_logging.hpp"  // Logging macros and utils
#include "interbotix_xs_driver/xs_common.hpp"   // Common variables and types
#include "interbotix_xs_driver/xs_driver.hpp"   // The InterbotixDriverXS class
```

```cmake
...
find_package(interbotix_xs_driver REQUIRED)
...
add_executable(your_executable)
ament_target_dependencies(your_executable interbotix_xs_driver ...)
...
```

Then create an `InterbotixDriverXS` object, providing the following in the order stated:
- Absolute filepath to the motor configs file
- Absolute filepath to the mode configs file
- A boolean indicating whether the Driver should write to the EEPROM on startup
- A string indicating the driver's logging level containing one of the following: "DEBUG", "INFO", "WARN", "ERROR", "FATAL"

This initialization would look something like below:

```c++
std::unique_ptr<InterbotixDriverXS> xs_driver = std::make_unique<InterbotixDriverXS>(
  filepath_motor_configs,
  filepath_mode_configs,
  write_eeprom_on_startup,
  logging_level);
```

See the package's source code for more details.
