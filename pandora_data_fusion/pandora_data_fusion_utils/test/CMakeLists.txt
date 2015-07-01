find_package(rostest REQUIRED)

########################  Utils  ####################################

catkin_add_gtest(utils_test unit/utils_test.cpp)
target_link_libraries(utils_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main
  )
