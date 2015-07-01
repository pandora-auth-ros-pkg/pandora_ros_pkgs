find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)

include_directories(
  ${pandora_testing_tools_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

########################  PoseFinderTest  ############################

catkin_add_gtest(pose_finder_test unit/pose_finder_test.cpp)
target_link_libraries(pose_finder_test
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${pandora_testing_tools_LIBRARIES}
  ${PROJECT_NAME}_pose_finder
  gtest_main
  )
