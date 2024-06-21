###############################################################################
# Macro
###############################################################################

macro(ecl_io_add_example name)
  set(target_name ecl_io_example_${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(${target_name}
    ${PROJECT_NAME}
    ${sockets_LIBRARIES}
  )
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME demo_${name})
  install(TARGETS ${target_name} RUNTIME DESTINATION lib/${PROJECT_NAME})
endmacro()

##############################################################################
# Examples
##############################################################################

ecl_io_add_example(sockets sockets.cpp)
ecl_io_add_example(socketpair socketpair.cpp)
ecl_io_add_example(poll poll.cpp)
