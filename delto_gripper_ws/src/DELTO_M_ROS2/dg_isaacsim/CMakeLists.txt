cmake_minimum_required(VERSION 3.8)
project(dg5f_isaacsim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY launch rviz urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
  script/dg5f_right_isaacsim_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
