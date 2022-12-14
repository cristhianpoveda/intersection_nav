cmake_minimum_required(VERSION 2.8.3)
project(intersection_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
)

#add_message_files(
#  FILES
#  InputBuffer.msg
#)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectStopSign.srv
  DetectUsers.srv
  MakeDecision.srv
)

add_action_files(
  DIRECTORY action 
  FILES IntersectionDriving.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES duckietown_msgs
   CATKIN_DEPENDS
   roscpp rospy
   std_msgs
   geometry_msgs
   message_runtime
   actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
