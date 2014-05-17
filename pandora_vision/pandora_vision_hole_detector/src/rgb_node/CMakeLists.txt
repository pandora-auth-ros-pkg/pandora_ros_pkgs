add_executable(rgb_node
  rgb_node.cpp
  rgb.cpp
  hole_detector.cpp
  )

target_link_libraries(rgb_node
  utils_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(rgb_node
  ${PROJECT_NAME}_gencfg
  vision_communications_gencpp
  ${catkin_EXPORTED_TARGETS}
  )
