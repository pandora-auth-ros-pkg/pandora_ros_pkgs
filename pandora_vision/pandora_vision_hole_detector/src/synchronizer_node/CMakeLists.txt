add_executable(synchronizer_node
  rgb_depth_synchronizer.cpp
  rgb_depth_synchronizer_node.cpp
  )

target_link_libraries(synchronizer_node
  utils_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(synchronizer_node
  ${PROJECT_NAME}_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
  )
