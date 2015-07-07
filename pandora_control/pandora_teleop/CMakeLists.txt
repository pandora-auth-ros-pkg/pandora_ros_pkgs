cmake_minimum_required(VERSION 2.8.3)
project(pandora_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  roslint
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(motors_keyop src/motors_keyop.cpp)
add_executable(motors_keyop_node src/motors_keyop_node.cpp)
target_link_libraries(motors_keyop_node motors_keyop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# To check lint : catkin_make roslint_pandora_motor_control
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/${PROJECT_NAME}/*.h
  src/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
