find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})

################################################################################
#                               Unit Tests                                     #
################################################################################

catkin_add_gtest(soft_obstacle_detector_test
  unit/soft_obstacle_detector_test.cpp
  )
target_link_libraries(soft_obstacle_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_soft_obstacle_detector
  gtest_main
  gtest
  )

catkin_add_gtest(fast_symmetry_detector_test
  unit/fast_symmetry_detector_test.cpp
  )
target_link_libraries(fast_symmetry_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_fast_symmetry_detector
  gtest_main
  gtest
  )

catkin_add_gtest(barrel_detector_test
  unit/barrel_detector_test.cpp
  )
target_link_libraries(barrel_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_barrel_detector
  gtest_main
  gtest
  )
catkin_add_gtest(traversability_mask_test
  unit/traversability_mask_test.cpp)
target_link_libraries(traversability_mask_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_hard_obstacle_detector
  gtest_main
  gtest
  )

if (${PROJECT_NAME}_benchmark)
  add_rostest(benchmark/barrel_benchmark_test.launch)
endif()
