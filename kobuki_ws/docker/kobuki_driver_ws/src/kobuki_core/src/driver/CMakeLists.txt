##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# CONFIGURATIONS
##############################################################################

set(VERSION_FILE ${PROJECT_BINARY_DIR}/version_info.cpp)
configure_file(version_info.cpp.in ${VERSION_FILE} @ONLY)

##############################################################################
# LIBRARIES
##############################################################################

add_library(${PROJECT_NAME} ${SOURCES} ${VERSION_FILE})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    ecl_console::ecl_console
    ecl_converters::ecl_converters
    ecl_devices::ecl_devices
    ecl_geometry::ecl_geometry
    ecl_mobile_robot::ecl_mobile_robot
    ecl_sigslots::ecl_sigslots
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    SOVERSION ${${PROJECT_NAME}_VERSION}
    VERSION ${${PROJECT_NAME}_VERSION}
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
   RUNTIME DESTINATION bin
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
)

