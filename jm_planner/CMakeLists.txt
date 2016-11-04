cmake_minimum_required(VERSION 2.8.3)
project(jm_planner)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jm_planner
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
 add_executable(a_star src/a_star.cpp)
 target_link_libraries(a_star
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   opencv_nonfree
 )

