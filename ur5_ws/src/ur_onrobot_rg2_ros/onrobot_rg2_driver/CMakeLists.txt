
cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg2_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  ur_msgs
)

add_service_files(
  FILES
  GripperState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  std_msgs 
  ur_msgs
)

include_directories(
  launch
  srv
  ${catkin_INCLUDE_DIRS}
)