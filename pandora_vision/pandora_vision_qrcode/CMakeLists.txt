cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_qrcode)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules(ZBAR zbar)

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
    INCLUDE_DIRS
      include
    LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qrcode_node
    src/qrCode_detection.cpp
    src/qrCode_detector.cpp
    src/qrCode_node.cpp
)
add_dependencies(qrcode_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(qrcode_node ${catkin_LIBRARIES} zbar)

#~ To check lint : catkin_make roslint_pandora_vision_qrcode
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  include/qrCode_detection.h
  include/qrCode_detector.h
  src/qrCode_detector.cpp
  src/qrCode_detection.cpp
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
