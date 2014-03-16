cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_face)

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

add_executable(face_node 
    src/face_node.cpp
    src/face_detection.cpp
    src/face_detector.cpp
    src/skin_detector.cpp
    src/time_calculator.cpp 
)
target_link_libraries(face_node ${catkin_LIBRARIES} )
add_dependencies(face_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

########################### modelTRainer ############################
if(MODEL_ENABLE_TRAIN)
  add_executable(model_train 
                  src/model_trainer.cpp
  )
  target_link_libraries(model_train ${catkin_LIBRARIES} )
endif()

#~ To check lint : catkin_make roslint_pandora_vision_face
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            

  include/face_detection.h
  include/face_detector.h
  include/skin_detection.h
  include/skin_detector.h
  include/time_calculator.h
  src/face_detector.cpp
  src/face_detection.cpp
  src/skin_detector.cpp
  src/skin_detection.cpp
  src/time_calculator.cpp
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
