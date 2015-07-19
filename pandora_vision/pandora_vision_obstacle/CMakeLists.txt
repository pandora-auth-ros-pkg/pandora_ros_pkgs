cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_obstacle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  roslint
  dynamic_reconfigure
  cv_bridge
  sensor_msgs
  nav_msgs
  pcl_ros
  pandora_vision_msgs
  pandora_vision_common
  sensor_processor
  state_manager
  state_manager_msgs
  pandora_cmake_tools
  )

generate_dynamic_reconfigure_options(
  config/pandora_vision_obstacle/soft_obstacle_cfg.cfg
  config/pandora_vision_obstacle/hard_obstacle_cfg.cfg
  config/pandora_vision_obstacle/elevation_map.cfg
  config/pandora_vision_obstacle/barrel_node.cfg
  )

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      nodelet
      pluginlib
      dynamic_reconfigure
      cv_bridge
      pcl_ros
      sensor_msgs
      nav_msgs
      pandora_vision_msgs
      pandora_vision_common
      sensor_processor
      state_manager
      state_manager_msgs
    INCLUDE_DIRS
    LIBRARIES
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  )

############################# fast symmetry detector #########################
add_library(${PROJECT_NAME}_fast_symmetry_detector
  src/barrel_detection/fast_symmetry_detector.cpp
  src/barrel_detection/parameters.cpp
)
add_dependencies(${PROJECT_NAME}_fast_symmetry_detector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_fast_symmetry_detector
  ${catkin_LIBRARIES}
)

############################### obstacle preprocessor ##############################
add_library(${PROJECT_NAME}_obstacle_preprocessor
  src/obstacle_preprocessor.cpp
  )
target_link_libraries(${PROJECT_NAME}_obstacle_preprocessor
  ${catkin_LIBRARIES}
  )

############################### obstacle postprocessor ##############################
add_library(${PROJECT_NAME}_obstacle_postprocessor
  src/obstacle_postprocessor.cpp
  )
target_link_libraries(${PROJECT_NAME}_obstacle_postprocessor
  ${catkin_LIBRARIES}
  )

############################### barrel processor ##############################
add_library(${PROJECT_NAME}_barrel_processor
  src/barrel_detection/barrel_processor.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_barrel_detector
)

add_dependencies(${PROJECT_NAME}_barrel_processor
  ${PROJECT_NAME}_barrel_detector)

############################### barrel detector ##############################
add_library(${PROJECT_NAME}_barrel_detector
  src/barrel_detection/barrel_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_detector
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_fast_symmetry_detector
)
# target_link_libraries(${PROJECT_NAME}_barrel_processor
  # ${catkin_LIBRARIES}
  # ${PROJECT_NAME}_fast_symmetry_detector
  # )

############################### barrel handler ##############################
add_library(${PROJECT_NAME}_barrel_handler
  src/barrel_detection/barrel_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_barrel_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_obstacle_preprocessor
  ${PROJECT_NAME}_obstacle_postprocessor
  ${PROJECT_NAME}_barrel_processor
  )

############################### soft obstacle detector ##############################
add_library(${PROJECT_NAME}_soft_obstacle_detector
  src/soft_obstacle_detection/soft_obstacle_detector.cpp
  )
target_link_libraries(${PROJECT_NAME}_soft_obstacle_detector
  ${catkin_LIBRARIES}
  )

############################### soft obstacle processor ##############################
add_library(${PROJECT_NAME}_soft_obstacle_processor
  src/soft_obstacle_detection/soft_obstacle_processor.cpp
  )
target_link_libraries(${PROJECT_NAME}_soft_obstacle_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_soft_obstacle_detector
  )

############################### soft obstacle handler ##############################
add_library(${PROJECT_NAME}_soft_obstacle_handler
  src/soft_obstacle_detection/soft_obstacle_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_soft_obstacle_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_obstacle_preprocessor
  ${PROJECT_NAME}_obstacle_postprocessor
  ${PROJECT_NAME}_soft_obstacle_processor
  )

########################### hard obstacle preprocessor ################

add_library(${PROJECT_NAME}_hard_obstacle_preprocessor
  src/hard_obstacle_detection/hard_obstacle_preprocessor.cpp
  )
target_link_libraries(${PROJECT_NAME}_hard_obstacle_preprocessor
  ${catkin_LIBRARIES}
  )

############################### hard obstacle postprocessor ##############################
add_library(${PROJECT_NAME}_hard_obstacle_postprocessor
  src/hard_obstacle_detection/hard_obstacle_postprocessor.cpp
  )
target_link_libraries(${PROJECT_NAME}_hard_obstacle_postprocessor
  ${catkin_LIBRARIES}
  )

############################### hard obstacle detector ##############################
add_library(${PROJECT_NAME}_hard_obstacle_detector
  src/hard_obstacle_detection/hard_obstacle_detector.cpp
  src/hard_obstacle_detection/traversability_mask.cpp
  )
target_link_libraries(${PROJECT_NAME}_hard_obstacle_detector
  ${catkin_LIBRARIES}
  )

############################### hard obstacle processor ##############################
add_library(${PROJECT_NAME}_hard_obstacle_processor
  src/hard_obstacle_detection/hard_obstacle_processor.cpp
  )
target_link_libraries(${PROJECT_NAME}_hard_obstacle_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_hard_obstacle_detector
  )

############################ hard obstacle handler ##############################
add_library(${PROJECT_NAME}_hard_obstacle_handler
  src/hard_obstacle_detection/hard_obstacle_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_hard_obstacle_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_hard_obstacle_preprocessor
  ${PROJECT_NAME}_hard_obstacle_postprocessor
  ${PROJECT_NAME}_hard_obstacle_processor
  )

############################ hard obstacle node ##########################
add_executable(${PROJECT_NAME}_hard_obstacle_node
  src/hard_obstacle_detection/hard_obstacle_node.cpp
  )
add_dependencies(${PROJECT_NAME}_hard_obstacle_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_hard_obstacle_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_hard_obstacle_handler
  )

add_library(${PROJECT_NAME}_dummy_lib
  src/hard_obstacle_detection/dummy_processor.cpp)

target_link_libraries(${PROJECT_NAME}_dummy_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_hard_obstacle_detector
)
add_executable(${PROJECT_NAME}_dummy
  src/hard_obstacle_detection/dummy_main.cpp
)
target_link_libraries(${PROJECT_NAME}_dummy
  ${PROJECT_NAME}_dummy_lib
    )

############################ barrel node ###############################
add_executable(${PROJECT_NAME}_barrel_node
  src/barrel_detection/barrel_node.cpp
)
add_dependencies(${PROJECT_NAME}_barrel_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_barrel_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_barrel_handler
  )

############################ soft obstacle node ###############################
add_executable(${PROJECT_NAME}_soft_obstacle_node
  src/soft_obstacle_detection/soft_obstacle_node.cpp
  )
add_dependencies(${PROJECT_NAME}_soft_obstacle_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_soft_obstacle_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_soft_obstacle_handler
  )

#~ To check lint : catkin_make roslint_pandora_vision_obstacle

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/*/*.h
  src/*/*.cpp
  test/*/*.cpp
  )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################################### Testing ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
