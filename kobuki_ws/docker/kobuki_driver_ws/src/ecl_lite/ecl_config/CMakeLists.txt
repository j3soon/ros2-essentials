##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_config)

##############################################################################
# Find Packages
##############################################################################

# Turn -isystem off. Actually like to see warnings from underlying packages
# and regardless, have run into trouble because of the ordering it induces.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()
ecl_detect_platform()

##############################################################################
# Header Library Targets
##############################################################################

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Exports
##############################################################################

# Downstream CMake3 compatibility 
ament_export_targets(${PROJECT_NAME})
# Downstream CMake2 compatibility
ament_export_include_directories(include)

ament_package()

