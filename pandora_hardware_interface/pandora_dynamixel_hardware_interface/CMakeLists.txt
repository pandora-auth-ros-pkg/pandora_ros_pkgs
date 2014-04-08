cmake_minimum_required(VERSION 2.8.3)
project(pandora_dynamixel_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
  dynamixel_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    roslint
    sensor_msgs
    dynamixel_msgs
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

#~ set(CMAKE_BUILD_TYPE Debug)

add_executable(
  joint_states_wrapper
  src/joint_states_wrapper.cpp)
target_link_libraries(
  joint_states_wrapper
  ${catkin_LIBRARIES})

# To check lint : catkin_make roslint_pandora_dynamixel_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  src/joint_states_wrapper.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
