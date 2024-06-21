###############################################################################
# SETUP
###############################################################################

add_executable(ecl_detect_endian detect_endian.cpp)
target_link_libraries(ecl_detect_endian ${PROJECT_NAME})
set_target_properties(ecl_detect_endian PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
install(TARGETS ecl_detect_endian RUNTIME DESTINATION bin)

if(NOT MSVC)
  add_executable(ecl_detect_posix detect_posix.cpp)
  target_link_libraries(ecl_detect_posix ${PROJECT_NAME})
  set_target_properties(ecl_detect_posix PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
  install(TARGETS ecl_detect_posix RUNTIME DESTINATION bin)
endif()

