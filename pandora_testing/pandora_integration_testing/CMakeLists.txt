cmake_minimum_required(VERSION 2.8.3)
project(pandora_integration_testing)

find_package(catkin REQUIRED)

catkin_package(
  #DEPENDS
  #CATKIN_DEPENDS
  #INCLUDE_DIRS
  #LIBRARIES
  )

########################  Integration Tests  ########################

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #add_rostest(launch/data_fusion_agent_test.launch)
endif()

