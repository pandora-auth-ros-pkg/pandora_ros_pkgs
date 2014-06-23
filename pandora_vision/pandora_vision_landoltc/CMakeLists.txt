cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_landoltc)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

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
    urdf
)

generate_dynamic_reconfigure_options(
  cfg/landoltc_cfg.cfg
  cfg/landoltc3d_cfg.cfg
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
      urdf
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


########################### landoltcNode ############################
add_executable(landoltc_node 
              src/landoltc_node.cpp
              src/landoltc_detection.cpp
              src/landoltc_detector.cpp
              src/landoltc_parameters.cpp
)
target_link_libraries(landoltc_node ${catkin_LIBRARIES} )

add_dependencies(landoltc_node 
    pandora_vision_landoltc_gencfg
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

########################### landoltc3dNode ############################
add_executable(landoltc3d_node 
              src/landoltc3d_parameters.cpp
              src/landoltc3d_node.cpp
              src/landoltc3d_detection.cpp
              src/landoltc3d_detector.cpp
              
              
)
target_link_libraries(landoltc3d_node ${catkin_LIBRARIES} )

add_dependencies(landoltc3d_node
    pandora_vision_landoltc_gencfg     
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)


#~ To check lint : catkin_make roslint_pandora_vision_landoltc
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  include/landoltc3d_detector.h
  include/landoltc3d_detection.h
  include/landoltc_detector.h
  include/landoltc_detection.h
  include/landoltc_parameters.h
  include/landoltc3d_parameters.h
  src/landoltc_detector.cpp
  src/landoltc3d_detector.cpp
  src/landoltc3d_detection.cpp
  src/landoltc3d_node.cpp
  src/landoltc_detection.cpp
  src/landoltc_node.cpp
  src/landoltc_parameters.cpp
  src/landoltc3d_parameters.cpp
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
