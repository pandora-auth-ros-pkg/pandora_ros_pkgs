add_library(${PROJECT_NAME}_rgb
  rgb.cpp
  hole_detector.cpp)
target_link_libraries(${PROJECT_NAME}_rgb
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_rgb
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})


add_executable(rgb_node rgb_node.cpp)
target_link_libraries(rgb_node
  ${PROJECT_NAME}_rgb
  ${catkin_LIBRARIES})

