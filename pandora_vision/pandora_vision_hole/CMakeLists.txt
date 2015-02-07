cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hole)

# Uncomment for generating coverage statistics.
# CMAKE_BUILD_TYPE should not be set.
# CXX should be set to g++, NOT clang.
#
# Usage:
# step 1: cd into build directory
# step 2: lcov --directory . --zerocounters
# step 3: cd ..
# step 4: catkin_make
# step 5: catkin_make run_tests -DCMAKE_BUILD_TYPE=Coverage
# step 6: cd into build directory
# step 7: lcov --capture --directory . --output-file coverage.info
# step 8: genhtml coverage.info
#SET(CMAKE_CXX_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
#SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pandora_vision_msgs
  state_manager
  state_manager_msgs
  cv_bridge
  image_transport
  sensor_msgs
  dynamic_reconfigure
  pcl_ros
  message_filters
  roslint
  urdf
  )

generate_dynamic_reconfigure_options(
  cfg/depth_node_cfg/depth_cfg.cfg
  cfg/hole_fusion_node_cfg/hole_fusion_cfg.cfg
  cfg/hole_fusion_node_cfg/debug_cfg.cfg
  cfg/hole_fusion_node_cfg/filters_priority_cfg.cfg
  cfg/hole_fusion_node_cfg/filters_thresholds_cfg.cfg
  cfg/hole_fusion_node_cfg/general_cfg.cfg
  cfg/hole_fusion_node_cfg/validity_cfg.cfg
  cfg/rgb_node_cfg/rgb_cfg.cfg
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pandora_vision_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    pcl_ros
    message_filters
    urdf
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_utils
  )

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  )

################################ Utils library #################################
add_library(${PROJECT_NAME}_utils
  src/utils/blob_detection.cpp
  src/utils/bounding_box_detection.cpp
  src/utils/edge_detection.cpp
  src/utils/histogram.cpp
  src/utils/hole_filters.cpp
  src/utils/holes_conveyor.cpp
  src/utils/message_conversions.cpp
  src/utils/morphological_operators.cpp
  src/utils/noise_elimination.cpp
  src/utils/outline_discovery.cpp
  src/utils/parameters.cpp
  src/utils/timer.cpp
  src/utils/visualization.cpp
  src/utils/wavelets.cpp
  )

target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )

add_subdirectory(src/depth_node)
add_subdirectory(src/hole_fusion_node)
add_subdirectory(src/rgb_node)
add_subdirectory(src/synchronizer_node)



################################# ROSLINT ######################################
# Execute via lint : catkin_make roslint_{PACKAGE_NAME}

set(ROSLINT_CPP_OPTS
  "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  ## depth_node ##

  include/depth_node/depth.h
  include/depth_node/hole_detector.h

  src/depth_node/depth.cpp
  src/depth_node/depth_node.cpp
  src/depth_node/hole_detector.cpp


  ## utils ##

  include/utils/blob_detection.h
  include/utils/bounding_box_detection.h
  include/utils/defines.h
  include/utils/edge_detection.h
  include/utils/histogram.h
  include/utils/hole_filters.h
  include/utils/holes_conveyor.h
  include/utils/message_conversions.h
  include/utils/morphological_operators.h
  include/utils/noise_elimination.h
  include/utils/outline_discovery.h
  include/utils/parameters.h
  #include/utils/timer.h
  include/utils/visualization.h
  include/utils/wavelets.h

  src/utils/blob_detection.cpp
  src/utils/bounding_box_detection.cpp
  src/utils/defines.h
  src/utils/edge_detection.cpp
  src/utils/histogram.cpp
  src/utils/hole_filters.cpp
  src/utils/holes_conveyor.cpp
  src/utils/message_conversions.cpp
  src/utils/morphological_operators.cpp
  src/utils/noise_elimination.cpp
  src/utils/outline_discovery.cpp
  src/utils/parameters.cpp
  #src/utils/timer.cpp
  src/utils/visualization.cpp
  src/utils/wavelets.cpp


  ## hole_fusion_node ##

  include/hole_fusion_node/depth_filters.h
  include/hole_fusion_node/hole_merger.h
  include/hole_fusion_node/hole_fusion.h
  include/hole_fusion_node/hole_validation.h
  include/hole_fusion_node/hole_uniqueness.h
  include/hole_fusion_node/filters.h
  include/hole_fusion_node/filters_resources.h
  include/hole_fusion_node/planes_detection.h
  include/hole_fusion_node/rgb_filters.h

  src/hole_fusion_node/depth_filters.cpp
  src/hole_fusion_node/hole_merger.cpp
  src/hole_fusion_node/hole_fusion.cpp
  src/hole_fusion_node/hole_validation.cpp
  src/hole_fusion_node/hole_uniqueness.cpp
  src/hole_fusion_node/hole_fusion_node.cpp
  src/hole_fusion_node/filters.cpp
  src/hole_fusion_node/filters_resources.cpp
  src/hole_fusion_node/planes_detection.cpp
  src/hole_fusion_node/rgb_filters.cpp


  ## synchronizer_node ##

  include/synchronizer_node/rgb_depth_synchronizer.h

  src/synchronizer_node/rgb_depth_synchronizer.cpp
  src/synchronizer_node/rgb_depth_synchronizer_node.cpp

  ## rgb_node ##

  include/rgb_node/rgb.h
  include/rgb_node/hole_detector.h

  src/rgb_node/rgb.cpp
  src/rgb_node/hole_detector.cpp
  src/rgb_node/rgb_node.cpp


  ##################### tests #####################

  ##################### utils #####################
  test/unit/utils/bounding_box_detection_test.cpp
  test/unit/utils/blob_detection_test.cpp
  test/unit/utils/edge_detection_test.cpp
  test/unit/utils/hole_filters_test.cpp
  test/unit/utils/holes_conveyor_test.cpp
  test/unit/utils/message_conversions_test.cpp
  test/unit/utils/morphological_operators_test.cpp
  test/unit/utils/noise_elimination_test.cpp
  test/unit/utils/outline_discovery_test.cpp
  test/unit/utils/wavelets_test.cpp

  ################## depth node ###################
  test/unit/depth_node/hole_detector_test.cpp

  ################### rgb node ####################
  test/unit/rgb_node/hole_detector_test.cpp

  ############### hole fusion node ################
  test/unit/hole_fusion_node/depth_filters_test.cpp
  test/unit/hole_fusion_node/filters_resources_test.cpp
  test/unit/hole_fusion_node/hole_merger_test.cpp
  test/unit/hole_fusion_node/hole_validation_test.cpp
  test/unit/hole_fusion_node/hole_uniqueness_test.cpp
  test/unit/hole_fusion_node/planes_detection_test.cpp
  test/unit/hole_fusion_node/rgb_filters_test.cpp

  )

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})


##################################### Testing ##################################
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
