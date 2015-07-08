add_library(${PROJECT_NAME}_rgb_utils
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
target_link_libraries(${PROJECT_NAME}_rgb_utils
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_timer
  )
add_dependencies(${PROJECT_NAME}_rgb_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_library(${PROJECT_NAME}_rgb
  rgb.cpp
  rgb_hole_detector.cpp)
target_link_libraries(${PROJECT_NAME}_rgb
  ${PROJECT_NAME}_rgb_utils
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_rgb
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})


# add_executable(rgb_node rgb_node.cpp)
# target_link_libraries(rgb_node
  # ${PROJECT_NAME}_rgb
  # ${catkin_LIBRARIES})
