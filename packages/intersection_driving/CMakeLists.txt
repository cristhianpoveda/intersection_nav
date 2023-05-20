cmake_minimum_required(VERSION 2.8.3)
project(intersection_driving)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  intersection_msgs
  move_base_msgs
  geometry_msgs
  std_srvs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)