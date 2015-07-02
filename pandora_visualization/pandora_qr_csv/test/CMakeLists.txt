find_package(catkin REQUIRED roslib rostest pandora_testing_tools)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${pandora_testing_tools_INCLUDE_DIRS}
  )

########################  Unit Tests  ###############################

########################  geotiff_creator  ####################################

catkin_add_gtest(qr_csv_creator_test unit/qr_csv_creator_test.cpp)
target_link_libraries(qr_csv_creator_test
  ${catkin_LIBRARIES}
  ${pandora_testing_tools_LIBRARIES}
  qr_csv_creator
  gtest_main
  )
