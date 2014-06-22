cmake_minimum_required(VERSION 2.8.3)
project(pandora_integration_testing)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
  roslib
  vision_communications
  pandora_data_fusion_msgs
  pandora_navigation_msgs
  pandora_rqt_gui
  pandora_testing_tools
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
  rospy
  rostest
  roslib
  vision_communications
  pandora_data_fusion_msgs
  pandora_navigation_msgs
  pandora_rqt_gui
  pandora_testing_tools
  INCLUDE_DIRS
  LIBRARIES
  )

########################  Integration Tests  ########################

#add_rostest(launch/data_fusion_agent_test.launch)

