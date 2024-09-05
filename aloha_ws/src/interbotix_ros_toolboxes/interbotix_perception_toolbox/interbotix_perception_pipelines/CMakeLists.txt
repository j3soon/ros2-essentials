cmake_minimum_required(VERSION 3.5)
project(interbotix_perception_pipelines)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(interbotix_perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(
  PERCEPTION_PIPELINE_ROS_DEPENDENCIES
    ament_index_cpp
    rclcpp
    sensor_msgs
    interbotix_perception_msgs
    visualization_msgs
    pcl_ros
    std_srvs
)

add_executable(pointcloud_pipeline src/pointcloud_pipeline.cpp)
ament_target_dependencies(pointcloud_pipeline ${PERCEPTION_PIPELINE_ROS_DEPENDENCIES})

install(
  TARGETS
    pointcloud_pipeline
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
