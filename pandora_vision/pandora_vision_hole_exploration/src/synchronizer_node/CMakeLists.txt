add_executable(${PROJECT_NAME}_synchronizer_node
  rgb_depth_thermal_synchronizer.cpp
  rgb_depth_thermal_synchronizer_node.cpp
  )

target_link_libraries(${PROJECT_NAME}_synchronizer_node
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME}_synchronizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
