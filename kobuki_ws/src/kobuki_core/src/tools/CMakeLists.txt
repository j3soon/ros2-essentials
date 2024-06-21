###############################################################################
# Macro
###############################################################################

macro(kobuki_core_add_tool name)
  set(target_name kobuki_core_tool_${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(${target_name}
    ${PROJECT_NAME}
    ecl_command_line::ecl_command_line
    ecl_console::ecl_console
  )
  # if we want dashes
  string(REGEX REPLACE "_" "-" dashed_name kobuki-${name})
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME ${dashed_name})
  install(
    TARGETS
      ${target_name}
    RUNTIME DESTINATION
      bin
  )
endmacro()

###############################################################################
# Tools
###############################################################################

kobuki_core_add_tool(version_info)
kobuki_core_add_tool(simple_keyop)
