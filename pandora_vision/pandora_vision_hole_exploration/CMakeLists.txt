cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hole_exploration)

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
  pandora_vision_common
  pandora_vision_msgs
  sensor_processor
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
  pandora_cmake_tools
  )

generate_dynamic_reconfigure_options(
  cfg/depth_node_cfg/depth_cfg.cfg
  cfg/hole_fusion_node_cfg/hole_fusion_cfg.cfg
  cfg/hole_fusion_node_cfg/validity_cfg.cfg
  cfg/hole_fusion_node_cfg/debug_cfg.cfg
  #cfg/hole_fusion_node_cfg/filters_priority_cfg.cfg
  #cfg/hole_fusion_node_cfg/filters_thresholds_cfg.cfg
  cfg/hole_fusion_node_cfg/general_cfg.cfg
  cfg/rgb_node_cfg/rgb_cfg.cfg
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pandora_vision_common
    pandora_vision_msgs
    sensor_processor
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
   src/utils/parameters.cpp
   src/utils/rgb_parameters.cpp
   src/utils/depth_parameters.cpp
   src/utils/timer.cpp
   src/utils/visualization.cpp
   src/utils/message_conversions.cpp
   src/utils/holes_conveyor.cpp
   src/utils/haralickfeature_extractor.cpp
  )


target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}_utils  ${catkin_EXPORTED_TARGETS}
  )

add_subdirectory(src/depth_node)
add_subdirectory(src/hole_fusion_node)
add_subdirectory(src/rgb_node)
add_subdirectory(src/synchronizer_node)
#add_subdirectory(src/thermal_node)

################################# ROSLINT ######################################
# Execute via lint : catkin_make roslint_{PACKAGE_NAME}

set(ROSLINT_CPP_OPTS
  "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}


  ## utils ##

  include/utils/parameters.h
  include/utils/rgb_parameters.h
  include/utils/depth_parameters.h
  #include/utils/timer.h
  include/utils/visualization.h
  include/utils/message_conversions.h
  include/utils/defines.h
  include/utils/holes_conveyor.h
  include/utils/haralickfeature_extractor.h

  src/utils/parameters.cpp
  #src/utils/timer.cpp
  src/utils/visualization.cpp
  src/utils/message_conversions.cpp
  src/utils/holes_conveyor.cpp
  src/utils/haralickfeature_extractor.cpp


  ### hole_fusion_node ##

  include/hole_fusion_node/hole_fusion.h

  src/hole_fusion_node/hole_fusion.cpp
  src/hole_fusion_node/hole_fusion_node.cpp


  ## synchronizer_node ##

  include/synchronizer_node/rgb_depth_thermal_synchronizer.h

  src/synchronizer_node/rgb_depth_thermal_synchronizer.cpp
  src/synchronizer_node/rgb_depth_thermal_synchronizer_node.cpp

  ## rgb_node ##

  include/rgb_node/rgb_processor.h

  src/rgb_node/rgb_processor.cpp
  src/rgb_node/rgb_node.cpp

  ## depth_node ##

  include/depth_node/depth_processor.h

  src/depth_node/depth_processor.cpp
  src/depth_node/depth_node.cpp


  ##################### tests #####################

  ##################### utils #####################
  test/unit/utils/holes_conveyor_test.cpp
  test/unit/utils/message_conversions_test.cpp
  test/unit/utils/visualization_test.cpp

  #################### depth node ###################
  test/unit/depth_node/depth_processor_test.cpp

  #################### rgb node ####################
  test/unit/rgb_node/rgb_processor_test.cpp

  ################ hole fusion node ################
  test/unit/hole_fusion_node/hole_fusion_test.cpp

  )

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################################### Testing ##################################
if(CATKIN_ENABLE_TESTING)
  if(DOWNLOAD_TESTING_DATA)
    set(${PROJECT_NAME}_download_testing_data true)
  endif()
  if(FUNCTIONAL_TEST)
    set(${PROJECT_NAME}_functional_test true)
  endif()
  if(BENCHMARK)
    set(${PROJECT_NAME}_benchmark true)
  endif()
  add_subdirectory(test)
endif()
