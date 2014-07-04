cmake_minimum_required(VERSION 2.8.3)
project(pandora_visualization)

find_package(catkin REQUIRED COMPONENTS 
      roscpp
      visualization_msgs
      tf
      pandora_data_fusion_msgs
      sensor_msgs
      interactive_markers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS   
    CATKIN_DEPENDS
      roscpp
      visualization_msgs
      tf
      pandora_data_fusion_msgs
      sensor_msgs
      interactive_markers
    INCLUDE_DIRS
    LIBRARIES # TODO
)


add_executable(data_fusion_object_visualization src/data_fusion_object_visualization.cpp)
target_link_libraries(data_fusion_object_visualization ${catkin_LIBRARIES} )
add_dependencies(data_fusion_object_visualization  pandora_data_fusion_msgs_gencpp )

add_executable(temprature_visualization src/temprature_visualization.cpp)
target_link_libraries(temprature_visualization ${catkin_LIBRARIES} )

add_executable(
  interactive_tf_publisher_marker
  src/interactive_markers/interactive_tf_publisher_marker.cpp)
target_link_libraries(
  interactive_tf_publisher_marker
  ${catkin_LIBRARIES} )

add_executable(
  interactive_command_publisher_marker
  src/interactive_markers/interactive_command_publisher_marker.cpp)
target_link_libraries(
  interactive_command_publisher_marker
  ${catkin_LIBRARIES} )
