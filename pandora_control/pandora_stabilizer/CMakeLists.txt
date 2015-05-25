cmake_minimum_required(VERSION 2.8.3)
project(pandora_stabilizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  pandora_sensor_msgs
  roslint
  )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS
    rospy
    roscpp
    tf
    std_msgs
    geometry_msgs
    pandora_sensor_msgs
#  DEPENDS system_lib
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(
  ${PROJECT_NAME}_control_node
  src/stabilizer_control.cpp
  src/stabilizer_control_node.cpp
  )
target_link_libraries(
  ${PROJECT_NAME}_control_node
  ${catkin_LIBRARIES}
  )
add_dependencies(
  ${PROJECT_NAME}_control_node
  ${catkin_EXPORTED_TARGETS}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(pandora_testing_tools REQUIRED)

  include_directories(
  ${pandora_testing_tools_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

  add_rostest(launch/stabilizer_test.launch)

  FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
    RELATIVE ${PROJECT_SOURCE_DIR}
    test/*
    )
  list(REMOVE_ITEM ${PROJECT_NAME}_LINT_PYTHON
    RELATIVE ${PROJECT_SOURCE_DIR}
    *.pyc
    )
  roslint_python(${${PROJECT_NAME}_LINT_PYTHON})

endif()

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/pandora_stabilizer_control/stabilizer_control.h
  src/stabilizer_control_node.cpp
  src/stabilizer_control.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
roslint_add_test()
