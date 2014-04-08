cmake_minimum_required(VERSION 2.8.3)
project(pandora_motor_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  serial
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    roslint
    controller_manager
    serial
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#~ set(CMAKE_BUILD_TYPE Debug)

add_executable(
  motor_hardware_interface_node
  src/motor_hardware_interface.cpp
  src/motor_hardware_interface_node.cpp)
target_link_libraries(
  motor_hardware_interface_node
  ${catkin_LIBRARIES})
  
add_library(motor_serial src/motor_serial_interface.cpp)
target_link_libraries(motor_serial ${catkin_LIBRARIES}) 

add_executable(motor_test src/test.cpp)
target_link_libraries(motor_test motor_serial ${catkin_LIBRARIES}) 

# To check lint : catkin_make roslint_pandora_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/pandora_motor_hardware_interface/motor_hardware_interface.h
  src/motor_hardware_interface.cpp
  src/motor_hardware_interface_node.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
