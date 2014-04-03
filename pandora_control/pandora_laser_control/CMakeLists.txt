cmake_minimum_required(VERSION 2.8.3)
project(pandora_laser_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf 
  std_msgs
  geometry_msgs
  sensor_msgs
  roslint
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS 
    roscpp 
    tf 
    std_msgs
    geometry_msgs
    sensor_msgs
    roslint
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pandora_stabilizer_control_node
  src/pandora_stabilizer_control.cpp
  src/pandora_stabilizer_control_node.cpp)
target_link_libraries(
  pandora_stabilizer_control_node 
  ${catkin_LIBRARIES} )

# To check lint : catkin_make roslint_pandora_control
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/pandora_stabilizer_control/pandora_stabilizer_control.h
  src/pandora_stabilizer_control_node.cpp
  src/pandora_stabilizer_control.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
