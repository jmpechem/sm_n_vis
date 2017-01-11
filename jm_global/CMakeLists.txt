cmake_minimum_required(VERSION 2.8.3)
project(jm_global)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jm_global
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGNE3_INCLUDE_DIR}
)

 add_executable(jm_global_map src/jm_global_map.cpp src/global_map.cpp)
 target_link_libraries(jm_global_map
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   opencv_nonfree
 )

