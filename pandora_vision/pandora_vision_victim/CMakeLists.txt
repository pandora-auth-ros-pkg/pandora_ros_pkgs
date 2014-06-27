cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_victim)

#~ set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_common_msgs
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    roslint
    urdf
)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_common_msgs
      vision_communications
      state_manager
      state_manager_communications
      cv_bridge
      image_transport
      sensor_msgs
      roslint
      urdf
    INCLUDE_DIRS
      include
    LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(victim_node
    src/victim_node.cpp
    src/victim_detection.cpp
    src/victim_detector.cpp
    src/rgb_system_validator.cpp
    src/depth_system_validator.cpp
    src/face_detector.cpp
    src/channels_statistics_extractor.cpp
    src/edge_orientation_extractor.cpp
    src/haralickfeature_extractor.cpp
)
target_link_libraries(victim_node ${catkin_LIBRARIES} )
add_dependencies(victim_node
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

########################### victimTrainer ############################
if(VICTIM_ENABLE_TRAIN)
  add_executable(victim_train_node
                  src/rgb_system_validator.cpp
                  src/depth_system_validator.cpp
                  src/channels_statistics_extractor.cpp
                  src/edge_orientation_extractor.cpp
                  src/haralickfeature_extractor.cpp
                  src/training.cpp
  )
  target_link_libraries(victim_train_node ${catkin_LIBRARIES} )
endif()

########################### modelTRainer ############################
if(MODEL_ENABLE_TRAIN)
  add_executable(model_train
                  src/model_trainer.cpp
  )
  target_link_libraries(model_train ${catkin_LIBRARIES} )
endif()

#~ To check lint : catkin_make roslint_pandora_vision_victim
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  
  include/victim_detection.h
  include/victim_detector.h
  include/face_detector.h
  include/rgb_system_validator.h
  include/depth_system_validator.h
  include/channels_statistics_extractor.h
  include/edge_orientation_extractor.h
  include/haralickfeature_extractor.h
  src/victim_detection.cpp
  src/victim_detector.cpp
  src/face_detector.cpp
  src/rgb_system_validator.cpp
  src/depth_system_validator.cpp
  src/channels_statistics_extractor.cpp
  src/edge_orientation_extractor.cpp
  src/haralickfeature_extractor.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
