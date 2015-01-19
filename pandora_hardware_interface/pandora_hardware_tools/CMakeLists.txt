cmake_minimum_required(VERSION 2.8.3)
project(pandora_hardware_tools)
find_package(catkin REQUIRED COMPONENTS
  rospy
  state_manager_msgs
  pandora_sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    rospy
    state_manager_msgs
    pandora_sensor_msgs
#  DEPENDS
)

