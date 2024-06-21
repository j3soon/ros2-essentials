##############################################################################
# Files
##############################################################################

add_executable(demo_sigslots_lite sigslots.cpp)
target_link_libraries(demo_sigslots_lite ${PROJECT_NAME})
install(TARGETS demo_sigslots_lite RUNTIME DESTINATION lib/${PROJECT_NAME})
