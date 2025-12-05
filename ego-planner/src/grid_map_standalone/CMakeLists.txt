cmake_minimum_required(VERSION 3.0.2)
project(grid_map_standalone)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_filters
  eigen_conversions
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Grid map standalone node (using full grid_map implementation)
add_executable(grid_map_node
  src/grid_map_node.cpp
  src/grid_map.cpp
  src/raycast.cpp
)

target_link_libraries(grid_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
