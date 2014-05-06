find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  )

link_directories(${catkin_LIBRARIES_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

############## Unit Tests ####################

catkin_add_gtest(morphological_operators_test unit/utils/morphological_operators_test.cpp)
target_link_libraries(morphological_operators_test ${catkin_LIBRARIES} depth_node_lib gtest_main) 

############## Functional Tests ###############

if(TARGET tests)
  add_executable(hole_detector_test EXCLUDE_FROM_ALL functional/hole_detector_test.cpp)
  target_link_libraries(hole_detector_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(tests hole_detector_test)
endif()
#add_rostest_gtest(hole_detector_test functional/hole_detector_test.test functional/hole_detector_test.cpp)
add_rostest(functional/hole_detector_test.launch)
