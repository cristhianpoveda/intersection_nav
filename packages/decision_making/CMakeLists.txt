cmake_minimum_required(VERSION 2.8.3)
project(decision_making)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  intersection_msgs
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    intersection_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(decision_making_node src/main.cpp src/simple_problem.cpp)
target_include_directories(decision_making_node PRIVATE include)
#set_target_properties(${PROJECT_NAME}_decision_making_node PROPERTIES OUTPUT_NAME decision_making_node PREFIX "")
add_dependencies(decision_making_node ${${PROJECT_NAME}EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_making_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
