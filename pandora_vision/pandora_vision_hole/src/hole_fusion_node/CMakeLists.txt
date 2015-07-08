add_library(${PROJECT_NAME}_hole_fusion_utils
  utils/blob_detection.cpp
  utils/bounding_box_detection.cpp
  utils/edge_detection.cpp
  utils/histogram.cpp
  utils/hole_filters.cpp
  utils/holes_conveyor.cpp
  utils/message_conversions.cpp
  utils/morphological_operators.cpp
  utils/noise_elimination.cpp
  utils/outline_discovery.cpp
  utils/parameters.cpp
  utils/visualization.cpp
  utils/wavelets.cpp
  utils/image_matching.cpp
  )
target_link_libraries(${PROJECT_NAME}_hole_fusion_utils
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_timer
  )
add_dependencies(${PROJECT_NAME}_hole_fusion_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_library(${PROJECT_NAME}_filters
  filters.cpp
  filters_resources.cpp
  rgb_filters.cpp
  depth_filters.cpp
  planes_detection.cpp)
target_link_libraries(${PROJECT_NAME}_filters
  ${PROJECT_NAME}_hole_fusion_utils
  ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_hole_fusion
  hole_merger.cpp
  hole_uniqueness.cpp
  hole_validation.cpp
  hole_fusion.cpp)
target_link_libraries(${PROJECT_NAME}_hole_fusion
  ${PROJECT_NAME}_hole_fusion_utils
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hole_fusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

# add_executable(hole_fusion_node
  # hole_fusion_node.cpp)
# target_link_libraries(hole_fusion_node
  # ${PROJECT_NAME}_hole_fusion
  # ${catkin_LIBRARIES})
