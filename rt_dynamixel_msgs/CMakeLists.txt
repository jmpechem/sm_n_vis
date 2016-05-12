cmake_minimum_required(VERSION 2.8.3)
project(rt_dynamixel_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  JointState.msg
  JointSet.msg
  Error.msg
)

add_service_files(
  FILES
  MotorSetting.srv
  ModeSetting.srv
)
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
