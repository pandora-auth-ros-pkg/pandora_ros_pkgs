cmake_minimum_required(VERSION 2.8.3)
project(pandora_motor_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  geometry_msgs
  roslint
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
    roslint
#  DEPENDS system_lib
)

#~ include_directories(${catkin_INCLUDE_DIRS})

# To check lint : catkin_make roslint_pandora_control
#~ set(ROSLINT_CPP_OPTS
  #~ "--filter=-build/include_what_you_use")
#~ 
#~ FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  #~ RELATIVE ${PROJECT_SOURCE_DIR}
#~ )
#~ roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
