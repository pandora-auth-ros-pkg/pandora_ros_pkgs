cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_orientation_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  pandora_sensor_msgs
  message_generation
  tf
  urdf
  roslint
  )

catkin_python_setup()

add_action_files(
  FILES
  MoveSensor.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
  pandora_sensor_msgs
    tf
    urdf
    message_runtime
#  DEPENDS system_lib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}_node
  src/sensor_orientation_controller.cpp
  )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencpp
  )

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
  RELATIVE ${PROJECT_SOURCE_DIR}
  scripts/**/*.py
  )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/*
  src/*
  )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
