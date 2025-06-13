cmake_minimum_required(VERSION 3.8)
project(dg5f_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(dg5f_both_gz_test_cpp src/dg5f_both_gz_test.cpp)
ament_target_dependencies(dg5f_both_gz_test_cpp rclcpp trajectory_msgs)

add_executable(dg5f_left_gz_test_cpp src/dg5f_left_gz_test.cpp)
ament_target_dependencies(dg5f_left_gz_test_cpp rclcpp trajectory_msgs)

add_executable(dg5f_right_gz_test_cpp src/dg5f_right_gz_test.cpp)
ament_target_dependencies(dg5f_right_gz_test_cpp rclcpp trajectory_msgs)

install(TARGETS 
  dg5f_both_gz_test_cpp
  dg5f_left_gz_test_cpp
  dg5f_right_gz_test_cpp
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
  script/dg5f_both_gz_test.py
  script/dg5f_left_gz_test.py
  script/dg5f_right_gz_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
