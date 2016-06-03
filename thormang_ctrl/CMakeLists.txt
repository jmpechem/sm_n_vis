cmake_minimum_required(VERSION 2.8.3)
project(thormang_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vrep_common
  std_msgs
  sensor_msgs
  geometry_msgs
  rt_dynamixel_msgs
  thormang_ctrl_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
file(GLOB BRIDGE_SRC
  "src/*.cpp"
)
file(GLOB WALKING_SRC
  "src/walking_ctrl/*.cpp"
)
file(GLOB UPPER_SRC
  "src/Upperbody_ctrl/*.cpp"
)
add_library(thormang_ctrl_lib
   ${BRIDGE_SRC}
   ${WALKING_SRC}
   ${UPPER_SRC}
)

add_executable(thormang_ctrl src/thormang_ctrl.cpp)
target_link_libraries(thormang_ctrl
   ${catkin_LIBRARIES}
   thormang_ctrl_lib
)
SET(CMAKE_BUILD_TYPE Debug)
