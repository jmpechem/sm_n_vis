cmake_minimum_required(VERSION 2.8.3)
project(sensoray826)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/sensoray826/
)

add_library(sensoray826
  src/sensoray826.cpp
)

file(GLOB_RECURSE HeaderFiles "include/*.h")
add_custom_target(sensoray826_header SOURCES ${HeaderFiles})

add_executable(ati_ft_sensor_node src/ati_ft_sensor.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(
#sensoray826_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
#)

## Specify libraries to link a library or executable target against


target_link_libraries(sensoray826
  -l826_64
  ${catkin_LIBRARIES}
)
target_link_libraries(ati_ft_sensor_node
  sensoray826
  ${catkin_LIBRARIES}
)

set(CMAKE_BUILD_TYPE Debug)
