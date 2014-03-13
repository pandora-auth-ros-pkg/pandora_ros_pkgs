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
