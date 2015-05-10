cmake_minimum_required(VERSION 2.8.3)
project(pandora_leddar_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  controller_manager
  controller_interface
  roslint
  pluginlib
  pandora_cmake_tools
  )

set(THIS_PACKAGE_INCLUDE_DIRS
  leddar_serial_interface/include
  leddar_usb_interface/include
  leddar_hardware_interface/include
  leddar_controllers/include
  )
  
catkin_package(
  INCLUDE_DIRS 
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES 
    leddar_sensor_controller
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    controller_manager
    controller_interface
    pluginlib
    roslint
  )

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_subdirectory(leddar_serial_interface)
add_subdirectory(leddar_usb_interface)
add_subdirectory(leddar_hardware_interface)
add_subdirectory(leddar_controllers)

# To check lint : catkin_make roslint_pandora_leddar_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  leddar_controllers/*.h
  ledddar_controllers/*.cpp
  leddar_hardware_interface/*.h
  leddar_hardware_interface/*.cpp
  leddar_serial_interface/leddar_serial_interface.h
  leddar_serial_interface/leddar_serial_interface.cpp
  leddar_serial_interface/leddar_serial_interface_demo.cpp
  leddar_usb_interface/leddar_usb_interface.h
  leddar_usb_interface/*.cpp
  )

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
