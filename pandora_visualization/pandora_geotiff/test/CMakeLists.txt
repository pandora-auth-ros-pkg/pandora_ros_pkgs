#~ find_package(roslib REQUIRED)
#~ find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)

include_directories(
  ${pandora_testing_tools_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_executable(data_fusion_mock EXCLUDE_FROM_ALL functional/data_fusion_mock_srv.cpp)
target_link_libraries(data_fusion_mock ${catkin_LIBRARIES})
add_dependencies(data_fusion_mock ${catkin_EXPORTED_TARGETS})

########################  Unit Tests  ###############################

########################  geotiff_creator  ####################################

catkin_add_gtest(geotiff_creator_test unit/geotiff_creator_test.cpp)
target_link_libraries(geotiff_creator_test 
  ${catkin_LIBRARIES}
  ${pandora_testing_tools_LIBRARIES}
  geotiff_creator
  gtest_main
  )


