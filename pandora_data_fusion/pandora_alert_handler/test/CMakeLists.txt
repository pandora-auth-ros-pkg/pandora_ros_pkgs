find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)

include_directories(
  ${pandora_testing_tools_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

########################  Tests  ####################################

########################  Unit Tests  ###############################

#########################  ObjectsTest  #############################

catkin_add_gtest(objects_test unit/objects_test.cpp)
target_link_libraries(objects_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_objects
  gtest_main
  )

########################  ObjectListTest  ###########################

catkin_add_gtest(object_list_test unit/object_list_test.cpp)
target_link_libraries(object_list_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_utils
  gtest_main
  )

########################  ObjectFactoryTest  ########################

catkin_add_gtest(object_factory_test unit/object_factory_test.cpp)
target_link_libraries(object_factory_test
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${pandora_testing_tools_LIBRARIES}
  ${PROJECT_NAME}_object_factory
  gtest_main
  )

#########################  VictimTest  ##############################

catkin_add_gtest(victim_test unit/victim_test.cpp)
target_link_libraries(victim_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_victim
  gtest_main
  )

#########################  VictimListTest  ##########################

catkin_add_gtest(victim_list_test unit/victim_list_test.cpp)
target_link_libraries(victim_list_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_victim_list
  gtest_main
  )

########################  VictimClustererTest  ######################

catkin_add_gtest(victim_clusterer_test unit/victim_clusterer_test.cpp)
target_link_libraries(victim_clusterer_test ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim_clusterer
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_victim
  gtest_main
  )

#########################  Functional Tests  ########################

catkin_download_test_data(
  ${PROJECT_NAME}_saloon.bag
  http://downloads.pandora.ee.auth.gr/bags/saloon_2014-04-27-20-30-41.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
  MD5 01603ce158575da859b8afff5b676bf9
  )

add_rostest(functional/subscriber_test.launch)
add_rostest(functional/alert_handler_static_test.launch)
add_rostest(functional/alert_handler_test.launch)

