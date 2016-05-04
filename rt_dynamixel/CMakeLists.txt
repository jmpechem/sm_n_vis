cmake_minimum_required(VERSION 2.8.3)
project(rt_dynamixel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosrt
  std_msgs
  rt_dynamixel_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rt_dynamixel
#  CATKIN_DEPENDS roscpp rosrt
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__
  include/rt_dynamixel/
)

add_library(rt_serial_port SHARED src/rt_serial_port.cpp)
add_library(rt_dynamixel_pro SHARED src/rt_dynamixel_pro.cpp)
add_library(rt_ros_service STATIC src/rt_ros_service.cpp)
add_library(dxl_lists STATIC src/dxl_lists.cpp)

add_executable(rt_dynamixel_node src/rt_dynamixel_node.cpp)

target_link_libraries(rt_serial_port
#  ${catkin_LIBRARIES}
  -lpthread
  -L/usr/xenomai/lib -lnative -lxenomai -lrtdm
)
target_link_libraries(rt_dynamixel_pro
#  ${catkin_LIBRARIES}
  -lpthread
  -L/usr/xenomai/lib -lnative -lxenomai -lrtdm
)

target_link_libraries(rt_ros_service
  ${catkin_LIBRARIES}
  -lpthread
  -L/usr/xenomai/lib -lnative -lxenomai -lrtdm
  rt_serial_port
  rt_dynamixel_pro
)

target_link_libraries(rt_dynamixel_node
  ${catkin_LIBRARIES}
  -lpthread
  -L/usr/xenomai/lib -lnative -lxenomai -lrtdm
  rt_serial_port
  rt_dynamixel_pro
  rt_ros_service
  dxl_lists
)

set(CMAKE_BUILD_TYPE Debug)

