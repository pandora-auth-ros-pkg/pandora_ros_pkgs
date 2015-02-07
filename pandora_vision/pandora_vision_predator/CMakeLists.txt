cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_predator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pandora_vision_msgs
  pandora_common_msgs
  state_manager
  state_manager_msgs
  roscpp
  image_transport
  sensor_msgs
  roslint
  urdf
  dynamic_reconfigure
)


find_package(OpenCV REQUIRED)
find_package(open_tld REQUIRED)

include_directories(include
  ${open_tld_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  config/predator_cfg.cfg
)

catkin_package(
 #INCLUDE_DIRS include
  DEPENDS
    OpenCV
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    pandora_vision_msgs
    pandora_common_msgs
    state_manager
    state_manager_msgs
    sensor_msgs
    urdf
    dynamic_reconfigure
    open_tld
)

add_executable(predator_node
      src/predator_node.cpp
      src/predator.cpp)

target_link_libraries(predator_node
  ${open_tld_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

add_dependencies(predator_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#~ To check lint : catkin_make roslint_pandora_vision_predator
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  src/predator_node.cpp
  include/predator_node.h


)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
