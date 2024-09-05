cmake_minimum_required(VERSION 3.5)
project(interbotix_xs_driver LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(DEPENDENCIES
  dynamixel_workbench_toolbox
)

# Generate version header file from package.xml
include(cmake/Version.cmake)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

add_library(
  ${PROJECT_NAME} SHARED
    src/xs_driver.cpp
    src/xs_logging.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  FILES
    ${PATH_TO_VERSION_OUTPUT}
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  FILES
    package.xml
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_dependencies(
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
