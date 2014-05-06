cmake_minimum_required(VERSION 2.8.3)
project(pandora_xmega_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
  std_msgs
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    power_supply_controller
    range_sensor_controller
  CATKIN_DEPENDS
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
  include
  ${catkin_INCLUDE_DIRS}
)

#~ set(CMAKE_BUILD_TYPE Debug)

add_library(
  xmega_interface
  src/pandora_xmega_hardware_interface/battery_sensor.cpp
  src/pandora_xmega_hardware_interface/range_sensor.cpp
  src/pandora_xmega_hardware_interface/xmega_serial_interface.cpp)
target_link_libraries(
  xmega_interface
  ${catkin_LIBRARIES})

add_executable(
  xmega_hardware_interface_node
  src/pandora_xmega_hardware_interface/xmega_hardware_interface.cpp
  src/pandora_xmega_hardware_interface/xmega_hardware_interface_node.cpp)
target_link_libraries(
  xmega_hardware_interface_node
  xmega_interface
  ${catkin_LIBRARIES})

add_executable(
  xmega_test
  src/pandora_xmega_hardware_interface/test.cpp)
target_link_libraries(
  xmega_test
  xmega_interface
  ${catkin_LIBRARIES})

add_library(
  power_supply_controller
  src/pandora_xmega_controllers/power_supply_controller.cpp)
target_link_libraries(
  power_supply_controller
  ${catkin_LIBRARIES})

add_library(
  range_sensor_controller
  src/pandora_xmega_controllers/range_sensor_controller.cpp)
target_link_libraries(
  range_sensor_controller
  ${catkin_LIBRARIES})

# To check lint : catkin_make roslint_pandora_xmega_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  #~ include/pandora_xmega_hardware_interface/battery_sensor.h
  #~ include/pandora_xmega_hardware_interface/default_sensor.h
  include/pandora_xmega_hardware_interface/power_supply_interface.h
  #~ include/pandora_xmega_hardware_interface/range_sensor.h
  include/pandora_xmega_hardware_interface/range_sensor_interface.h
  #~ include/pandora_xmega_hardware_interface/sensor_base.h
  include/pandora_xmega_hardware_interface/xmega_hardware_interface.h
  include/pandora_xmega_controllers/power_supply_controller.h
  include/pandora_xmega_controllers/range_sensor_controller.h
  #~ include/pandora_xmega_hardware_interface/xmega_serial_interface.h
  #~ src/battery_sensor.cpp
  #~ src/range_sensor.cpp
  #~ src/test.cpp
  src/pandora_xmega_hardware_interface/xmega_hardware_interface.cpp
  src/pandora_xmega_controllers/power_supply_controller.cpp
  src/pandora_xmega_controllers/range_sensor_controller.cpp
  #~ src/xmega_serial_interface.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
