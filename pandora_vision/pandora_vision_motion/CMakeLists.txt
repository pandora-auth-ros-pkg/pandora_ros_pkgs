cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_motion)

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

add_executable(motion_node 
    src/motion_node.cpp
    src/motion_detection.cpp
    src/motion_detector.cpp
)
target_link_libraries(motion_node ${catkin_LIBRARIES} )
add_dependencies(motion_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)
