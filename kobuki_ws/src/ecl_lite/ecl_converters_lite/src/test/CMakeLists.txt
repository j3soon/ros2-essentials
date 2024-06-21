##############################################################################
# Files
##############################################################################

ament_add_gtest(test_byte_array byte_array.cpp)
target_link_libraries(test_byte_array ${PROJECT_NAME})
