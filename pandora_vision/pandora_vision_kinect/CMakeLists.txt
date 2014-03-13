cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_kinect)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    pcl_ros
)

generate_dynamic_reconfigure_options(
  cfg/kinect.cfg
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
      pcl_ros
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


########################### kinectNode ############################
add_library(kinect_node 
  src/kinect.cpp
)
target_link_libraries(kinect_node 
  ${catkin_LIBRARIES}
)
add_dependencies(kinect_node 
  pandora_vision_kinect_gencfg
)                        

add_executable(pandora_kinect_node 
  src/kinect_node.cpp
  src/hole_detector.cpp
  src/hole_filters.cpp
  src/planes_detection.cpp
  src/noise_elimination.cpp
  src/edge_detection.cpp
  src/blob_detection.cpp
  src/bounding_box_detection.cpp
  src/morphological_operators.cpp
  src/visualization.cpp
  src/parameters.cpp
  src/timer.cpp
)
set (KINECT_COMPILE_FLAGS "-Wno-deprecated") 

target_link_libraries(pandora_kinect_node 
  kinect_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
) 

set_target_properties(pandora_kinect_node 
  PROPERTIES COMPILE_FLAGS 
  ${KINECT_COMPILE_FLAGS}
)

add_dependencies(pandora_kinect_node 
  pandora_vision_kinect_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
) 
