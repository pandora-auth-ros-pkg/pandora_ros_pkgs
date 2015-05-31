add_library(${PROJECT_NAME}_rgb_processor
  rgb_processor.cpp)
target_link_libraries(${PROJECT_NAME}_rgb_processor
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_rgb_processor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_rgb_preprocessor
  rgb_preprocessor.cpp)
target_link_libraries(${PROJECT_NAME}_rgb_preprocessor
  ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_rgb_postprocessor
  rgb_postprocessor.cpp)
target_link_libraries(${PROJECT_NAME}_rgb_postprocessor
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_rgb_postprocessor
  ${catkin_EXPORTED_TARGETS}
  )

add_library(${PROJECT_NAME}_rgb_handler
  rgb_handler.cpp)
target_link_libraries(${PROJECT_NAME}_rgb_handler
  ${PROJECT_NAME}_rgb_processor
  ${PROJECT_NAME}_rgb_preprocessor
  ${PROJECT_NAME}_rgb_postprocessor
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_rgb_node rgb_node.cpp)
target_link_libraries(${PROJECT_NAME}_rgb_node
  ${PROJECT_NAME}_rgb_handler
  ${catkin_LIBRARIES})

