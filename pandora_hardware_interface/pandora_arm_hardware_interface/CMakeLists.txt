cmake_minimum_required(VERSION 2.8.3)
project(pandora_arm_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  roscpp
  roslint
  controller_manager
  controller_interface
  sensor_msgs
  pluginlib
)

set(THIS_PACKAGE_INCLUDE_DIRS
  arm_controllers/include
  arm_hardware_interface/include
  arm_usb_interface/include
)

add_message_files(
  FILES
  Co2Msg.msg
  ThermalMeanMsg.msg
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    co2_sensor_controller
    thermal_sensor_controller
  CATKIN_DEPENDS
    roscpp
    message_generation
    roscpp
    roslint
    controller_manager
    controller_interface
    sensor_msgs
    pluginlib
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(arm_controllers)
add_subdirectory(arm_hardware_interface)
add_subdirectory(arm_usb_interface)

# To check lint : catkin_make roslint_pandora_arm_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")
  
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  arm_controllers/*.h
  arm_controllers/*.cpp
  arm_hardware_interface/*.h
  arm_hardware_interface/*.cpp
  arm_usb_interface/*.h
  arm_usb_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
