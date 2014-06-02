cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_motion)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    roslint
)


generate_dynamic_reconfigure_options(
  config/motion_cfg.cfg
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      vision_communications
      state_manager
      state_manager_communications
      cv_bridge
      image_transport
      sensor_msgs
      dynamic_reconfigure
      roslint
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_node 
    src/motion_node.cpp
    src/motion_detection.cpp
    src/motion_detector.cpp
    src/motion_parameters.cpp
)
target_link_libraries(motion_node ${catkin_LIBRARIES} )
add_dependencies(motion_node
    pandora_vision_motion_gencfg
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

#~ To check lint : catkin_make roslint_pandora_vision_motion
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  include/motion_detection.h
  include/motion_detector.h
  include/motion_parameters.h
  src/motion_detector.cpp
  src/motion_detection.cpp
  src/motion_parameters.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

