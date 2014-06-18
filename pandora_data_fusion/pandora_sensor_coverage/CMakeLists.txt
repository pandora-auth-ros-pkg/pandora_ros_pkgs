cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_coverage)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf 
  roslint
  pandora_alert_handler
  state_manager
  state_manager_communications
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS 
  roscpp
  tf 
  roslint
  pandora_alert_handler
  state_manager
  state_manager_communications
  INCLUDE_DIRS 
  include
  LIBRARIES 
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

########################  sensor  ###################################

add_library(sensor
  src/sensor.cpp)
target_link_libraries(sensor
  ${catkin_LIBRARIES}
  ros_tf_listener
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

