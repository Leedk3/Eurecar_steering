cmake_minimum_required(VERSION 2.8.3)
project(usrg_path_tracking)

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

#set( hearders
#  include/usrg_path_tracking_node.cpp/quintic_polynomials.hpp
#)

#set( sources
#  src/usrg_path_tracking_node.cpp
#)

#add_executable(${PROJECT_NAME}_node ${sources} ${headers})

add_executable(${PROJECT_NAME}_node src/usrg_path_tracking_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

#Hear start GTSAM , PCL, OpenCV dependancy

find_package(GTSAM REQUIRED QUIET)
#find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS cloud_msgs
#  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
#	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

