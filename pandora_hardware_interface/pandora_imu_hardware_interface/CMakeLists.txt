cmake_minimum_required(VERSION 2.8.3)
project(pandora_imu_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  tf
  controller_manager
  serial
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    tf
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
  imu_hardware_interface_node
  src/imu_serial_interface.cpp
  src/ahrs_serial_interface.cpp
  src/imu_hardware_interface.cpp
  src/imu_hardware_interface_node.cpp)
target_link_libraries(
  imu_hardware_interface_node
  ${catkin_LIBRARIES})

add_executable(
  trax_ahrs_configuration_node
  src/ahrs_serial_interface.cpp
  src/trax_ahrs_configuration_node.cpp)
target_link_libraries(
  trax_ahrs_configuration_node
  ${catkin_LIBRARIES})


# To check lint : catkin_make roslint_pandora_imu_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  include/*.h
  src/*.cpp
  src/*.cpp
  src/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
