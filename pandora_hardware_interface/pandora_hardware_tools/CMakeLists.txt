cmake_minimum_required(VERSION 2.8.3)
project(pandora_hardware_tools)
find_package(catkin REQUIRED COMPONENTS
  rospy
  controllers_and_sensors_communications
  state_manager_communications
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    rospy
    controllers_and_sensors_communications
#  DEPENDS
)

