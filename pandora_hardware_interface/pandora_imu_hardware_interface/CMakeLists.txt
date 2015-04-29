cmake_minimum_required(VERSION 2.8.3)
project(pandora_imu_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  dynamic_reconfigure
  controller_manager
  controller_interface
  tf
  serial
  pandora_sensor_msgs
  pluginlib
)

generate_dynamic_reconfigure_options(imu_hardware_interface/config/ImuHardwareInterface.cfg)

SET(THIS_PACKAGE_INCLUDE_DIRS
  imu_com_interface/include
  imu_hardware_interface/include
  imu_controllers/include
  )


include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    imu_rpy_controller
  CATKIN_DEPENDS
    roscpp
    controller_manager
    controller_interface
    tf
    serial
    pandora_sensor_msgs
    pluginlib
)


add_subdirectory(imu_com_interface)
add_subdirectory(imu_hardware_interface)
add_subdirectory(imu_controllers)

#~ set(CMAKE_BUILD_TYPE Debug)

# To check lint : catkin_make roslint_pandora_imu_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  include/*.h
  src/*.cpp
  src/*.cpp
  src/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
