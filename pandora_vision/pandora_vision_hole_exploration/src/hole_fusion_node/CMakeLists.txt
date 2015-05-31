add_library(${PROJECT_NAME}_hole_fusion
  hole_fusion.cpp)
target_link_libraries(${PROJECT_NAME}_hole_fusion
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hole_fusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_hole_fusion_node
  hole_fusion_node.cpp)
target_link_libraries(${PROJECT_NAME}_hole_fusion_node
  ${PROJECT_NAME}_hole_fusion
  ${catkin_LIBRARIES})

