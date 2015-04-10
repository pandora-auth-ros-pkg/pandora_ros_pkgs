cmake_minimum_required(VERSION 2.8.3)
project(pandora_motor_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
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
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(pandora_teleop_key src/pandora_teleop_key.cpp )
target_link_libraries(pandora_teleop_key ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# To check lint : catkin_make roslint_pandora_motor_control
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  src/pandora_teleop_key.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
