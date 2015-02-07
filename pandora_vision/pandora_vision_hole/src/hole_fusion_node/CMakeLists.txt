add_library(${PROJECT_NAME}_filters
  filters.cpp
  filters_resources.cpp
  rgb_filters.cpp
  depth_filters.cpp
  planes_detection.cpp)
target_link_libraries(${PROJECT_NAME}_filters
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_hole_fusion
  hole_merger.cpp
  hole_uniqueness.cpp
  hole_validation.cpp
  hole_fusion.cpp)
target_link_libraries(${PROJECT_NAME}_hole_fusion
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hole_fusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(hole_fusion_node
  hole_fusion_node.cpp)
target_link_libraries(hole_fusion_node
  ${PROJECT_NAME}_hole_fusion
  ${catkin_LIBRARIES})

