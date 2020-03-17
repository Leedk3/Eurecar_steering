cmake_minimum_required(VERSION 2.8.3)
project(load_waypoint)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  tf
  cv_bridge
#  pcl_ros
#  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set( hearders
  include/load_waypoint/load_waypoint.hpp
)

set( sources
  src/load_waypoint_node.cpp
)

add_executable(${PROJECT_NAME}_node src/load_waypoint_node.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


