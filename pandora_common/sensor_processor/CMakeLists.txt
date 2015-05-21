cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  state_manager
  state_manager_msgs
  roslint
  )

add_message_files(
  FILES ProcessorLogInfo.msg
  )

generate_messages(
  DEPENDENCIES std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    state_manager
    state_manager_msgs
  INCLUDE_DIRS
    include
  )

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(pandora_testing_tools REQUIRED)

  add_library(${PROJECT_NAME}_dummy_preprocessor src/dummy_preprocessor.cpp)
  add_dependencies(${PROJECT_NAME}_dummy_preprocessor
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(${PROJECT_NAME}_dummy_preprocessor
    ${catkin_LIBRARIES}
    )

  add_library(${PROJECT_NAME}_dummy_processor src/dummy_processor.cpp)
  add_dependencies(${PROJECT_NAME}_dummy_processor
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(${PROJECT_NAME}_dummy_processor
    ${catkin_LIBRARIES}
    )

  add_library(${PROJECT_NAME}_dummy_postprocessor src/dummy_postprocessor.cpp)
  add_dependencies(${PROJECT_NAME}_dummy_postprocessor
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(${PROJECT_NAME}_dummy_postprocessor
    ${catkin_LIBRARIES}
    )

  add_library(${PROJECT_NAME}_dummy_handler src/dummy_handler.cpp)
  add_dependencies(${PROJECT_NAME}_dummy_handler
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(${PROJECT_NAME}_dummy_handler
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_dummy_preprocessor
    ${PROJECT_NAME}_dummy_processor
    ${PROJECT_NAME}_dummy_postprocessor
    )

  add_executable(${PROJECT_NAME}_dummy_processor_node src/dummy_processor_node.cpp)
  target_link_libraries(${PROJECT_NAME}_dummy_processor_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_dummy_handler
    )

  add_rostest(test/sensor_processor_node.launch)

endif()
