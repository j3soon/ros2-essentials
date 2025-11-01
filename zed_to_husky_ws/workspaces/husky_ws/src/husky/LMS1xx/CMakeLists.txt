cmake_minimum_required(VERSION 3.8)
project(lms1xx)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf   DESTINATION share/${PROJECT_NAME})

ament_package()