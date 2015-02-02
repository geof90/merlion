cmake_minimum_required(VERSION 2.8.3)
project(merlion_hw1)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
add_message_files(
   FILES
   state.msg
)
add_service_files(
  FILES
  move_robot.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  LIBRARIES merlion_hw1
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
  message_runtime
)
include_directories(
   ${catkin_INCLUDE_DIRS}
)
