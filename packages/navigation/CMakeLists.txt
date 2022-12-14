cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  intersection_msgs
  move_base_msgs
)

catkin_package()
