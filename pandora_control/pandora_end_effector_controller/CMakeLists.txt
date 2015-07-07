cmake_minimum_required(VERSION 2.8.3)
project(pandora_end_effector_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf
  urdf
  roslint
  pandora_linear_actuator_controller
  pandora_sensor_orientation_controller
  )

catkin_python_setup()

add_action_files(
  FILES
  MoveEndEffector.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  )

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    tf
    urdf
    message_runtime
#  DEPENDS system_lib
  )

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
  RELATIVE ${PROJECT_SOURCE_DIR}
  src/**/*.py
  )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})
