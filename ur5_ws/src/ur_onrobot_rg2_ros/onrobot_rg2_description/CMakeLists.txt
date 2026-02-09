cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg2_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  config
  launch
  meshes
  urdf
  ${catkin_INCLUDE_DIRS}
)