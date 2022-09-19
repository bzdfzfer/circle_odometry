cmake_minimum_required(VERSION 2.8.3)
project(circle_odometry)

add_compile_options(-std=c++14)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  tf_conversions
  pcl_conversions
  pcl_ros
  rosbag
)

find_package(PCL 1.8 REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(YAML-CPP REQUIRED)


catkin_package(
    # INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    LIBRARIES kitti-pcl
    CATKIN_DEPENDS roscpp rospy std_msgs
#     DEPENDS system_lib
)
message(" eigen 3 include dir:  -------------- ${EIGEN3_INCLUDE_DIR}" )
message(" pcl include dir:  -------------- ${PCL_INCLUDE_DIRS}" )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# LIBRARIES
add_library(cfm  
  src/histogram.cpp 
  src/line_feature.cpp
  src/plane_extraction.cpp
  src/plane_pairing.cpp
  src/solver.cpp
  src/circle_odometry.cpp
  src/Visualizer.cpp
)

target_link_libraries(cfm
  ${PCL_LIBRARIES} 
  ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libyaml-cpp.so
)


# NODES
add_executable(rosbag_circle_odometry_node 
  nodes/rosbag_circle_odometry_node.cpp
)
target_link_libraries(rosbag_circle_odometry_node
  cfm
)


add_executable(circle_odometry_node 
  nodes/circle_odometry_node.cpp
)
target_link_libraries(circle_odometry_node
  cfm
)
