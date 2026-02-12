
cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg2_gazebo)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  config
  launch
  ${catkin_INCLUDE_DIRS}
)