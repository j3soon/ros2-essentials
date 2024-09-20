cmake_minimum_required(VERSION 3.5)
project(interbotix_tf_tools)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(tf_rebroadcaster SHARED
  src/tf_rebroadcaster.cpp
)
target_compile_definitions(tf_rebroadcaster
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  tf_rebroadcaster
    geometry_msgs
    rclcpp
    rclcpp_components
    tf2_msgs
    tf2_ros
)
target_link_libraries(tf_rebroadcaster yaml-cpp)
rclcpp_components_register_nodes(tf_rebroadcaster "interbotix_tf_tools::TFRebroadcaster")

install(
  TARGETS
    tf_rebroadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/static_trans_pub.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
