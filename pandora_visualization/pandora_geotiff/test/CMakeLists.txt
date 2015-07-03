
find_package(rostest REQUIRED)
find_package(roslib REQUIRED)
find_package(stdr_server REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${rostest_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  ${stdr_server_INCLUDE_DIRS}
  )

########################  Unit Tests  ###############################

########################  geotiff_creator  ####################################

catkin_add_gtest(geotiff_creator_test unit/geotiff_creator_test.cpp)
target_link_libraries(geotiff_creator_test
  ${catkin_LIBRARIES}
  ${rostest_LIBRARIES}
  ${stdr_server_LIBRARIES}
  ${roslib_LIBRARIES}
  creator
  gtest_main
  )

catkin_add_gtest(utilities unit/utilities.cpp)
target_link_libraries(utilities
  ${catkin_LIBRARIES}
  ${rostest_LIBRARIES}
  server
  gtest_main
  )
