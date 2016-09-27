cmake_minimum_required(VERSION 2.8.3)
project(move_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  interactive_markers
)
catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
 add_executable(move_model src/move_model.cpp)
 target_link_libraries(move_model
   ${catkin_LIBRARIES}
 )

 add_executable(custom_recog src/custom_recog.cpp)
 target_link_libraries(custom_recog
   ${catkin_LIBRARIES}
 )

