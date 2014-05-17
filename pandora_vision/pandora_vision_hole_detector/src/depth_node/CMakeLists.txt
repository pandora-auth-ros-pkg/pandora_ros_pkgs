add_executable(depth_node
  depth.cpp
  depth_node.cpp
  hole_detector.cpp
  )

target_link_libraries(depth_node
  utils_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(depth_node
  ${PROJECT_NAME}_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
  )
