add_library(${PROJECT_NAME}_depth_processor
  depth_processor.cpp)
target_link_libraries(${PROJECT_NAME}_depth_processor
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_depth_processor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_depth_preprocessor
  depth_preprocessor.cpp)
target_link_libraries(${PROJECT_NAME}_depth_preprocessor
  ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_depth_postprocessor
  depth_postprocessor.cpp)
target_link_libraries(${PROJECT_NAME}_depth_postprocessor
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_depth_postprocessor
  ${catkin_EXPORTED_TARGETS}
  )

add_library(${PROJECT_NAME}_depth_handler
  depth_handler.cpp)
target_link_libraries(${PROJECT_NAME}_depth_handler
  ${PROJECT_NAME}_depth_processor
  ${PROJECT_NAME}_depth_preprocessor
  ${PROJECT_NAME}_depth_postprocessor
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_depth_node depth_node.cpp)
target_link_libraries(${PROJECT_NAME}_depth_node
  ${PROJECT_NAME}_depth_handler
  ${catkin_LIBRARIES})
