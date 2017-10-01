cmake_minimum_required(VERSION 2.8.3)
project(my_robot_description)

find_package(catkin REQUIRED COMPONENTS
  rviz
  tf
  urdf
  xacro
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
