###############################################################################
# Macro
###############################################################################

macro(kobuki_core_add_demo name)
  set(target_name kobuki_core_demo_${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(${target_name}
    ${PROJECT_NAME}
    ecl_command_line::ecl_command_line
    ecl_console::ecl_console
  )
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME demo_${name})
  install(
    TARGETS
      ${target_name}
    RUNTIME DESTINATION
      lib/${PROJECT_NAME}
  )
endmacro()

###############################################################################
# Tests
###############################################################################

kobuki_core_add_demo(buttons)
kobuki_core_add_demo(chirp)
kobuki_core_add_demo(custom_logging)
kobuki_core_add_demo(data_stream)
kobuki_core_add_demo(log_levels)
kobuki_core_add_demo(raw_data_stream)
kobuki_core_add_demo(simple_loop)
kobuki_core_add_demo(velocity_commands)
