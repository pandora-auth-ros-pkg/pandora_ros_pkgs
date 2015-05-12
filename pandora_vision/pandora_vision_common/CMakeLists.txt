cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  pandora_common_msgs
  sensor_processor
  urdf
  roslint
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    pandora_common_msgs
    sensor_processor
    urdf
  INCLUDE_DIRS
    include
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
