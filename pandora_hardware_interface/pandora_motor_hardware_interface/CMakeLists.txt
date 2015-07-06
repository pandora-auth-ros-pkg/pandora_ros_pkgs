cmake_minimum_required(VERSION 2.8.3)
project(pandora_motor_hardware_interface)
#~ set(CMAKE_BUILD_TYPE Debug)

find_package(PkgConfig REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  roslint
  joint_limits_interface
  controller_manager
  controller_interface
  serial
  pluginlib
  urdf
  angles
  tf
  pandora_sensor_msgs
  pandora_cmake_tools
  dynamic_reconfigure
)

add_message_files(
  FILES
   KinodynamicCommand.msg
   )

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
  )

set(THIS_PACKAGE_INCLUDE_DIRS
    motor_controllers/include
    motor_hardware_interface/include
    epos_interface/include
)

generate_dynamic_reconfigure_options(
  motor_hardware_interface/cfg/torqueControl.cfg
  motor_controllers/config/frictionCoefficient.cfg
)


## -- Link Against gsl library using either pkgconfig or hard-given paths
## -- Use this method if gsl is not installed under system paths
## - ROSBUILD architecture holds pkgconfig paths so be aware!
## - Better use the set() command to set include/lib directorries
###############################################################################
pkg_check_modules(GSL REQUIRED gsl)
#set(GSL_INCLUDE_DIRS ~/external_libraries/gsl-1.16_install_asd/include)
#set(GSL_LIBRARY_DIRS ~/external_libraries/gsl-1.16_install_asd/lib)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})
###############################################################################


catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  DEPENDS
    libgsl

  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    pandora_sensor_msgs
    joint_limits_interface
    controller_manager
    controller_interface
    serial
    pluginlib
    urdf
    angles
    tf
    dynamic_reconfigure
    message_runtime
  LIBRARIES
    skid_steer_drive_controller
    skid_steer_velocity_controller
    skid_steer_torque_controller
)


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
