cmake_minimum_required(VERSION 3.8)
project(citysim)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY media  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()