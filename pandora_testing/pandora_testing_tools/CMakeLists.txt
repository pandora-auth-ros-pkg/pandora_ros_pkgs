cmake_minimum_required(VERSION 2.8.3)
project(pandora_testing_tools)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  tf
  rostest
  rosbag
  message_generation 
  actionlib_msgs 
  std_msgs
  )

catkin_python_setup()

add_action_files(
  FILES
  ReplayBags.action
  )

generate_messages(
  DEPENDENCIES actionlib_msgs
  )

catkin_package(
  DEPENDS 
  CATKIN-DEPENDS 
  message_runtime 
  actionlib_msgs 
  std_msgs
  INCLUDE_DIRS 
  LIBRARIES 
  )

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(bag_player_for_tests 
  src/bag_player_for_tests.cpp 
  src/customized_player.cpp
  )
target_link_libraries(bag_player_for_tests ${catkin_LIBRARIES})
add_dependencies(bag_player_for_tests ${PROJECT_NAME}_gencpp)
