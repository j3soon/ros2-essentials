cmake_minimum_required(VERSION 3.12.0)
project(interbotix_footswitch_driver)

find_package(ament_cmake REQUIRED)
find_package(interbotix_footswitch_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(PkgConfig REQUIRED)
add_subdirectory(dependencies/hidapi)

include_directories(include)

set(ROS_DEPENDENCIES
  interbotix_footswitch_msgs
  rclcpp
  rclcpp_components
)

add_library(footswitch_driver
  src/footswitch_driver.cpp
)

ament_target_dependencies(footswitch_driver ${ROS_DEPENDENCIES})
target_link_libraries(footswitch_driver hidapi::hidraw)

add_executable(footswitch_driver_node
  src/footswitch_driver_node.cpp
)

target_link_libraries(footswitch_driver_node
  footswitch_driver
)

install(
  TARGETS footswitch_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    footswitch_driver_node
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
