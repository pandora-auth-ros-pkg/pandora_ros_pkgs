add_library(${PROJECT_NAME}_depth
  depth.cpp
  hole_detector.cpp)
target_link_libraries(${PROJECT_NAME}_depth
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_depth
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})


add_executable(depth_node depth_node.cpp)
target_link_libraries(depth_node
  ${PROJECT_NAME}_depth
  ${catkin_LIBRARIES})

