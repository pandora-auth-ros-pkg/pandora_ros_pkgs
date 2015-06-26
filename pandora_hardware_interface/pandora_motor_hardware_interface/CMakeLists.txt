cmake_minimum_required(VERSION 2.8.3)
project(pandora_motor_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
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
  std_msgs
  sensor_msgs
  pandora_sensor_msgs
  pandora_cmake_tools
)

find_package(PkgConfig)
pkg_check_modules(gsl REQUIRED gsl)

set(THIS_PACKAGE_INCLUDE_DIRS
    motor_controllers/include
    motor_hardware_interface/include
    epos_interface/include
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  DEPENDS
    gsl
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
    std_msgs
    pandora_sensor_msgs
  LIBRARIES
    skid_steer_drive_controller
#  DEPENDS
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(epos_interface)
add_subdirectory(motor_controllers)
add_subdirectory(motor_hardware_interface)


# To check lint : catkin_make roslint_pandora_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references, -runtime/arrays, -whitespace/empty_loop_body,-runtime/printf")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  motor_hardware_interface/*.h
  motor_hardware_interface/*.cpp
  motor_controllers/*.h
  motor_controllers/*.cpp
  epos_interface/*_gateway.h
  epos_interface/*_handler.h
  epos_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
