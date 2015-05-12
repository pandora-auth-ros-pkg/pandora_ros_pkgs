cmake_minimum_required(VERSION 2.8.3)
project(pandora_dynamixel_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  dynamixel_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    dynamixel_msgs
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

