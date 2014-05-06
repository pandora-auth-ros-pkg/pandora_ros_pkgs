cmake_minimum_required(VERSION 2.8.3)
project(pandora_linear_motor_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    roslint
    controller_manager
    controller_interface
    serial
#  LIBRARIES
#  DEPENDS
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

