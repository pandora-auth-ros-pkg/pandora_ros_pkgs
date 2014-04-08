cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_datamatrix)

#~ set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    state_manager
    state_manager_communications
    vision_communications
    cv_bridge
    image_transport
    sensor_msgs
    roslint
)

message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(DMTX REQUIRED)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      state_manager
      state_manager_communications
      vision_communications
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
  ${DMTX_INCLUDE_DIRS}
)

add_executable(datamatrix_node 
                  src/datamatrix_node.cpp
                  src/datamatrix_detection.cpp
                  src/datamatrix_detector.cpp
)
add_dependencies(datamatrix_node 
    ${catkin_EXPORTED_TARGETS}
    vision_communications_gencpp
)
target_link_libraries(datamatrix_node 
    ${catkin_LIBRARIES} 
    ${DMTX_LIBRARIES}
)

#~ To check lint : catkin_make roslint_pandora_vision_datamatrix
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  include/datamatrix_detection.h
  include/datamatrix_detector.h
  src/datamatrix_detector.cpp
  src/datamatrix_detection.cpp
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})


