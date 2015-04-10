cmake_minimum_required(VERSION 2.8.3)
project(pandora_end_effector_planner)

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
)

catkin_python_setup()

set(THIS_PACKAGE_INCLUDE_DIRS
    sensor_orientation_planner/include
    linear_movement_planner/include
)

add_action_files(
  FILES
  MoveSensor.action
  MoveLinear.action
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

#~ set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(sensor_orientation_planner)
add_subdirectory(linear_movement_planner)

# To check lint : catkin_make roslint_pandora_end_effector_planner
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  sensor_orientation_planner/*.h
  sensor_orientation_planner/*.cpp
  linear_movement_planner/*.h
  linear_movement_planner/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
     ${PROJECT_SOURCE_DIR}
     end_effector_planner/src/end_effector_planner_smach.py
     end_effector_planner/src/states.py
     end_effector_planner/src/topics.py
    )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})
