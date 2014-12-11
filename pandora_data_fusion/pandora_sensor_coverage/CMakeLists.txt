cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_coverage)

set(DCMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf 
  roslint
  pandora_alert_handler
  state_manager
  state_manager_msgs
  octomap_msgs
  octomap_ros
  )

find_package(octomap REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS 
  roscpp
  tf 
  roslint
  pandora_alert_handler
  state_manager
  state_manager_msgs
  INCLUDE_DIRS 
  include
  LIBRARIES 
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )
link_directories(
  ${OCTOMAP_LIBRARY_DIRS}
  )

########################  coverage checker  #########################

add_library(coverage_checker
  src/coverage_checker.cpp)
target_link_libraries(coverage_checker
  ${catkin_LIBRARIES}
  )

########################  space checker  ############################

#add_library(space_checker
  #src/space_checker.cpp)
#target_link_libraries(space_checker
  #${catkin_LIBRARIES}
  #coverage_checker
  #)

########################  surface checker  ##########################

add_library(surface_checker
  src/surface_checker.cpp)
target_link_libraries(surface_checker
  ${catkin_LIBRARIES}
  coverage_checker
  )

########################  sensor  ###################################

add_library(sensor
  src/sensor.cpp)
target_link_libraries(sensor
  ${catkin_LIBRARIES}
  ros_tf_listener
  surface_checker
  )

########################  sensor_coverage  ##########################

add_library(sensor_coverage
  src/sensor_coverage.cpp)
target_link_libraries(sensor_coverage
  ${catkin_LIBRARIES}
  sensor
  )

########################  sensor_coverage_node  #####################

add_executable(sensor_coverage_node 
  src/main.cpp)
target_link_libraries(sensor_coverage_node 
  ${catkin_LIBRARIES}
  sensor_coverage
  )

########################  roslint  ##################################

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
  include/sensor_coverage/*.h 
  src/*.cpp
  test/unit/*.cpp
  test/functional/*.cpp
  )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

########################  testing  ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

