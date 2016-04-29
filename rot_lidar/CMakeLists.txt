cmake_minimum_required(VERSION 2.8.3)
project(rot_lidar)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  dynamixel_controllers
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  laser_geometry
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
 add_executable(lidar_to_pcl src/lidar_to_pcl.cpp)
 target_link_libraries(lidar_to_pcl
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
 add_executable(sweep_n_assembly src/sweep_n_assembly.cpp)
 target_link_libraries(sweep_n_assembly
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
