cmake_minimum_required(VERSION 2.8.3)
project(pandora_motor_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
  geometry_msgs
  pluginlib
  urdf
  angles
  tf
  nav_msgs
)

set(THIS_PACKAGE_INCLUDE_DIRS
    motor_controllers/include
    motor_hardware_interface/include
    orangutan_interface/include
    epos_interface/include
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    controller_manager
    controller_interface
    serial
    geometry_msgs
    pluginlib
    urdf
    angles
    tf
    nav_msgs
  LIBRARIES
    skid_steer_drive_controller
#  DEPENDS
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(orangutan_interface)
add_subdirectory(motor_controllers)
add_subdirectory(motor_hardware_interface)
add_subdirectory(epos_interface)


# To check lint : catkin_make roslint_pandora_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -whitespace/comments, -runtime/references, -runtime/arrays, -whitespace/empty_loop_body")
  
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  motor_hardware_interface/*.h
  motor_hardware_interface/*.cpp
  #~ motor_controllers/*.h
  #~ motor_controllers/*.cpp
  orangutan_interface/*.h
  orangutan_interface/*.cpp
  epos_interface/*.h
  epos_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
