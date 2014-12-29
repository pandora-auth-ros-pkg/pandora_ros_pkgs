cmake_minimum_required(VERSION 2.8.3)
project(pandora_testing_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  rosbag
  message_generation
  actionlib_msgs
  actionlib
  std_msgs
  map_server
  )

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

catkin_python_setup()

add_action_files(
  FILES
    ReplayBags.action
  )

generate_messages(
  DEPENDENCIES
    actionlib_msgs
  )

catkin_package(
  #DEPENDS
  CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    actionlib
    std_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_map_loader
  )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_map_loader
  src/map_loader/map_loader.cpp
  )
target_link_libraries(${PROJECT_NAME}_map_loader
  ${catkin_LIBRARIES}
  yaml-cpp
  )

add_executable(bag_player_for_tests
  src/bag_player/bag_player_for_tests.cpp
  src/bag_player/customized_player.cpp
  )
target_link_libraries(bag_player_for_tests ${catkin_LIBRARIES})
add_dependencies(bag_player_for_tests ${${PROJECT_NAME}_EXPORTED_TARGETS})
