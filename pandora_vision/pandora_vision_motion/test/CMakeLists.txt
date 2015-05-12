find_package(rostest REQUIRED)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   )

################# Unit Tests #################
catkin_add_gtest(motion_detector_test
  unit/motion_detector_test.cpp
)
target_link_libraries(motion_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  gtest_main
)
