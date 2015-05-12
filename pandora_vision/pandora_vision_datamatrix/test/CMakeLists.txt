find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  rostest
  roslint
)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

################# Unit Tests #################
catkin_add_gtest(${PROJECT_NAME}_detector_test
  unit/datamatrix_detector_test.cpp
)
target_link_libraries(${PROJECT_NAME}_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  gtest_main
)
