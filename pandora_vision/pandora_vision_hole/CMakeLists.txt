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
  nodelet
  pluginlib
  message_generation
  rospy
  pandora_vision_msgs
  pandora_vision_common
  pandora_common_msgs
  distrib_msgs
  state_manager
  state_manager_msgs
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  dynamic_reconfigure
  pcl_ros
  message_filters
  roslint
  urdf
  pandora_cmake_tools
  )

generate_dynamic_reconfigure_options(
  cfg/thermal_node_cfg/thermal_cfg.cfg
  cfg/depth_node_cfg/depth_cfg.cfg
  cfg/hole_fusion_node_cfg/hole_fusion_cfg.cfg
  cfg/hole_fusion_node_cfg/debug_cfg.cfg
  cfg/hole_fusion_node_cfg/filters_priority_cfg.cfg
  cfg/hole_fusion_node_cfg/filters_thresholds_cfg.cfg
  cfg/hole_fusion_node_cfg/general_cfg.cfg
  cfg/hole_fusion_node_cfg/validity_cfg.cfg
  cfg/rgb_node_cfg/rgb_cfg.cfg
  )

add_message_files(
  FILES
  CandidateHoleMsg.msg
  CandidateHolesVectorMsg.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    message_runtime
    rospy
    pandora_vision_msgs
    pandora_vision_common
    pandora_common_msgs
    distrib_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    pcl_ros
    message_filters
    urdf
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_hole_fusion_utils
    ${PROJECT_NAME}_timer
  )

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  )

################################ Utils library #################################
add_library(${PROJECT_NAME}_timer
  src/utils/timer.cpp
  )
target_link_libraries(${PROJECT_NAME}_timer
  ${catkin_LIBRARIES}
  )

add_subdirectory(src/depth_node)
add_subdirectory(src/hole_fusion_node)
add_subdirectory(src/rgb_node)
add_subdirectory(src/synchronizer)
add_subdirectory(src/thermal_node)

# Download extra files not needed when building
# but necessary when running the package.
# If download fails, the build also fails.
download_extra_data(
  ${PROJECT_NAME}_download_backprojection_images
  EXTRA_FILES walls/extra_files.yaml
  )

if(DOWNLOAD_DATA OR FUNCTIONAL_TEST OR BENCHMARK)
# Download extra files, not needed when building
# and not necessary to run any executable.
# To download run: catkin_make download_extra_data
  download_data(
    ${PROJECT_NAME}_simple_pcl_frame.bag
    http://downloads.pandora.ee.auth.gr/bags/simple_pcl_frame.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
    MD5 4304760d90c53b80c3b42a4c148379e5
    EXCLUDE_FROM_ALL
    )

  download_data(
    ${PROJECT_NAME}_simple_pcl_frame_0.bag
    http://downloads.pandora.ee.auth.gr/bags/simple_pcl_frame_0.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
    MD5 d7d56db4b5d121aa8ffe75c7a41ff690
    EXCLUDE_FROM_ALL
    OPTIONAL
    )
endif()
################################# ROSLINT ######################################
# Execute via lint : catkin_make roslint_{PACKAGE_NAME}

#set(ROSLINT_CPP_OPTS
  #"--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")

set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  src/*.cpp

  ##################### tests #####################

  ##################### utils #####################
  #test/unit/utils/bounding_box_detection_test.cpp
  #test/unit/utils/blob_detection_test.cpp
  #test/unit/utils/edge_detection_test.cpp
  #test/unit/utils/hole_filters_test.cpp
  #test/unit/utils/holes_conveyor_test.cpp
  #test/unit/utils/message_conversions_test.cpp
  #test/unit/utils/morphological_operators_test.cpp
  #test/unit/utils/noise_elimination_test.cpp
  #test/unit/utils/outline_discovery_test.cpp
  #test/unit/utils/wavelets_test.cpp

  ################## depth node ###################
  #test/unit/depth_node/hole_detector_test.cpp

  ################### rgb node ####################
  #test/unit/rgb_node/hole_detector_test.cpp

  ############### hole fusion node ################
  #test/unit/hole_fusion_node/depth_filters_test.cpp
  #test/unit/hole_fusion_node/filters_resources_test.cpp
  #test/unit/hole_fusion_node/hole_merger_test.cpp
  #test/unit/hole_fusion_node/hole_validation_test.cpp
  #test/unit/hole_fusion_node/hole_uniqueness_test.cpp
  #test/unit/hole_fusion_node/planes_detection_test.cpp
  #test/unit/hole_fusion_node/rgb_filters_test.cpp

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
