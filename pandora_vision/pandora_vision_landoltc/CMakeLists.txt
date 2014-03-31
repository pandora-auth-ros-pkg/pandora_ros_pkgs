cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_landoltc)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    roslint
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
      roslint
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(landoltc_node 
              src/landoltc_node.cpp
						  src/landoltc_detection.cpp
              src/landoltc_detector.cpp
)
target_link_libraries(landoltc_node ${catkin_LIBRARIES} )

add_dependencies(landoltc_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

#~ To check lint : catkin_make roslint_pandora_vision_landoltc
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  include/landoltc_detector.h
  include/landoltc_detection.h
  src/landoltc_detector.cpp
  src/landoltc_detection.cpp
  src/landoltc_node.cpp
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
