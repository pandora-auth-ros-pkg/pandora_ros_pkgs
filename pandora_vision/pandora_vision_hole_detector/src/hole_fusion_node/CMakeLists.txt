add_executable(hole_fusion_node
  hole_fusion.cpp
  hole_fusion_node.cpp
  hole_merger.cpp
  hole_uniqueness.cpp
  hole_validation.cpp
  filters.cpp
  filters_resources.cpp
  rgb_filters.cpp
  depth_filters.cpp
  planes_detection.cpp
  )

target_link_libraries(hole_fusion_node
  utils_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(hole_fusion_node
  ${PROJECT_NAME}_gencfg
  pandora_vision_msgs_gencpp
  ${catkin_EXPORTED_TARGETS}
  )
