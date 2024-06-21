##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_core)

# Turn -isystem off. Actually like to see warnings from underlying packages
# and regardless, have run into trouble because of the ordering it induces.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake_ros REQUIRED)

find_package(ecl_build REQUIRED)
find_package(ecl_command_line REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_console REQUIRED)
find_package(ecl_converters REQUIRED)
find_package(ecl_devices REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_threads REQUIRED)
find_package(ecl_mobile_robot REQUIRED)
find_package(ecl_sigslots REQUIRED)
find_package(ecl_time REQUIRED)

##############################################################################
# Platform Configuration
##############################################################################

ament_package_xml()
ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Exports
##############################################################################

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
    ecl_config
    ecl_console
    ecl_converters
    ecl_devices
    ecl_geometry
    ecl_mobile_robot
    ecl_sigslots
    ecl_threads
    ecl_time
    )

install(
    DIRECTORY include/
    DESTINATION include
)
ament_export_include_directories(include)
# TODO: deprecate this line post 'ardent' (been fixed upstream)
ament_export_libraries(${PROJECT_NAME})
ament_package()
