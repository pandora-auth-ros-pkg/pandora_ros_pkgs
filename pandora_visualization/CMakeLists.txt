cmake_minimum_required(VERSION 2.8.3)
project(pandora_visualization)

find_package(catkin REQUIRED COMPONENTS 
      roscpp
      visualization_msgs
      tf
      pandora_navigation_communications 
      pandora_data_fusion_msgs
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
      pandora_navigation_communications 
      pandora_data_fusion_msgs
    INCLUDE_DIRS 
    LIBRARIES # TODO
)

add_executable(navigation_visualization src/navigation_visualization.cpp)
target_link_libraries(navigation_visualization ${catkin_LIBRARIES} )
add_dependencies(navigation_visualization  pandora_navigation_communications_gencpp )

add_executable(data_fusion_object_visualization src/data_fusion_object_visualization.cpp)
target_link_libraries(data_fusion_object_visualization ${catkin_LIBRARIES} )
add_dependencies(data_fusion_object_visualization  pandora_data_fusion_msgs_gencpp )
