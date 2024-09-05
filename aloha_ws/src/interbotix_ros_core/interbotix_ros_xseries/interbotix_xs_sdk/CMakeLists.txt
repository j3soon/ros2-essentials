cmake_minimum_required(VERSION 3.5)
project(interbotix_xs_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(interbotix_xs_driver REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(ROS_DEPENDENCIES
  ament_index_cpp
  rclcpp
  sensor_msgs
  urdf
  interbotix_xs_msgs
  interbotix_xs_driver
)

###########
## Build ##
###########

## Declare a C++ executable
## Specify libraries to link a library or executable target against
add_executable(xs_sdk src/xs_sdk.cpp src/xs_sdk_obj.cpp)
ament_target_dependencies(xs_sdk ${ROS_DEPENDENCIES})
target_link_libraries(xs_sdk yaml-cpp)

#############
## Install ##
#############

install(
  TARGETS xs_sdk
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/xs_sdk_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
