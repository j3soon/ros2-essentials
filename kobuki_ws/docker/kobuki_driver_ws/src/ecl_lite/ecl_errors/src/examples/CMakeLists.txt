###############################################################################
# Macro
###############################################################################

macro(ecl_errors_add_example name)
  set(target_name ecl_errors_example_${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(${target_name}
    ${PROJECT_NAME}
  )
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME demo_${name})
  if(CMAKE_COMPILER_IS_GNUCXX)
    set_target_properties(${target_name} PROPERTIES COMPILE_FLAGS "-Wno-unused-local-typedefs")
  endif()
  install(TARGETS ${target_name} RUNTIME DESTINATION lib/${PROJECT_NAME})
endmacro()

###############################################################################
# Examples
###############################################################################

ecl_errors_add_example(errors)
