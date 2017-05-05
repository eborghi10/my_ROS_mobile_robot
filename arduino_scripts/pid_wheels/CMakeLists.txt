cmake_minimum_required(VERSION 2.8.3)
project(pid_wheels)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  robot_msgs
  roslaunch
)

roslaunch_add_file_check(launch)

add_action_files(
   DIRECTORY action
   FILES PID.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs   
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs robot_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(PID_Server src/ControllerServer.cpp)
target_link_libraries(PID_Server ${catkin_LIBRARIES})
add_dependencies(PID_Server ${tutorial_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(PID_Client src/ControllerClient.cpp)
target_link_libraries(PID_Client ${catkin_LIBRARIES})
add_dependencies(PID_Client ${tutorial_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
