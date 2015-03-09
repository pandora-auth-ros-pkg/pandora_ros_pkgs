cmake_minimum_required(VERSION 2.8.3)
project(pandora_linear_motor_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
)

set(THIS_PACKAGE_INCLUDE_DIRS
    jrk_interface/include
    linear_motor_hardware_interface/include
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    controller_manager
    controller_interface
    serial
#  LIBRARIES
#  DEPENDS
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(jrk_interface)
add_subdirectory(linear_motor_hardware_interface)

# To check lint : catkin_make roslint_pandora_linear_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")
  
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  linear_motor_hardware_interface/*.h 
  linear_motor_hardware_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
