cmake_minimum_required(VERSION 3.8)
project(dg5f_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
# delto_tcp 라이브러리 정의
add_library(delto_tcp SHARED
  src/delto_developer_TCP.cpp
)

target_include_directories(delto_tcp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(delto_tcp
  rclcpp
)

# dg5f_driver 라이브러리 정의
add_library(dg5f_driver SHARED
  src/system_interface.cpp
)

target_include_directories(dg5f_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 로컬 라이브러리 링크 
target_link_libraries(dg5f_driver delto_tcp)

# ROS2 의존성은 ament_target_dependencies로 처리
ament_target_dependencies(dg5f_driver
  rclcpp
  pluginlib
  hardware_interface
)


add_executable(dg5f_operator_driver src/dg5f_operator_driver.cpp)
ament_target_dependencies(dg5f_operator_driver rclcpp trajectory_msgs sensor_msgs)

install(TARGETS 
  dg5f_operator_driver
  DESTINATION lib/${PROJECT_NAME})

  install(
  PROGRAMS 
  script/dg5f_left_test.py
  script/dg5f_right_test.py
  DESTINATION lib/${PROJECT_NAME}
)

  target_include_directories(dg5f_operator_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# install(
#   PROGRAMS 
#   script/dg5f_test.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# 설치 규칙
install(
  TARGETS dg5f_driver delto_tcp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface dg5f_driver.xml)

install(
  FILES dg5f_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
