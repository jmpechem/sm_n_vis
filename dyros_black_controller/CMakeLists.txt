cmake_minimum_required(VERSION 2.8.3)
project(dyros_black_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rt_dynamixel_msgs
  sensor_msgs
  std_msgs
  vrep_common
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dyros_black_controller_node src/dyros_black_controller_node.cpp)

target_link_libraries(dyros_black_controller_node
  ${catkin_LIBRARIES}
)

