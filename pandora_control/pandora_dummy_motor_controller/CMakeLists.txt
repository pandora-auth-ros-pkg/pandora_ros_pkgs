cmake_minimum_required(VERSION 2.8.3)
project(pandora_dummy_motor_controller)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  sensor_msgs
)

catkin_package(
  #~ INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    roslint
    std_msgs
    sensor_msgs
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

#~ set(CMAKE_BUILD_TYPE Debug)

add_executable(
  dummy_motor_controller
  src/dummy_motor_controller.cpp)
target_link_libraries(
  dummy_motor_controller
  ${catkin_LIBRARIES})

# To check lint : catkin_make roslint_dummy_motor_controller
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -whitespace/comments")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/dummy_motor_controller/dummy_motor_controller.h
  src/dummy_motor_controller.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
