cmake_minimum_required(VERSION 2.8.3)
project(pandora_xmega_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
  sensor_msgs
  pluginlib
)

set(THIS_PACKAGE_INCLUDE_DIRS
  xmega_controllers/include
  xmega_hardware_interface/include
  xmega_serial_interface/include
)

add_message_files(
  FILES
  BatteryMsg.msg
  )

generate_messages(
  DEPENDENCIES
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    battery_controller
    range_sensor_controller
  CATKIN_DEPENDS
    message_generation
    roscpp
    roslint
    controller_manager
    controller_interface
    serial
    std_msgs
    pluginlib
#  DEPENDS
)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(xmega_controllers)
add_subdirectory(xmega_hardware_interface)
add_subdirectory(xmega_serial_interface)

#~ set(CMAKE_BUILD_TYPE Debug)

# To check lint : catkin_make roslint_pandora_xmega_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -whitespace/comments, -runtime/references")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  xmega_controllers/*.h
  xmega_controllers/*.cpp
  xmega_hardware_interface/*.h
  xmega_hardware_interface/*.cpp
  xmega_serial_interface/*.h
  xmega_serial_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
