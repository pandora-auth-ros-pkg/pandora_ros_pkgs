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

catkin_add_gtest(discrete_wavelet_transform_test
  unit/discrete_wavelet_transform_test.cpp
  )
target_link_libraries(discrete_wavelet_transform_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_discrete_wavelet_transform
  gtest_main
  gtest
  )

catkin_add_gtest(soft_obstacle_detector_test
  unit/soft_obstacle_detector_test.cpp
  )
target_link_libraries(soft_obstacle_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_soft_obstacle_detector
  gtest_main
  gtest
  )
