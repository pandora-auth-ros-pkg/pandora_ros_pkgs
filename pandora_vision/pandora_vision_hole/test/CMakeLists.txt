###############################  Unit Tests ####################################

#################################### utils #####################################

catkin_add_gtest(outline_discovery_test
  unit/utils/outline_discovery_test.cpp)
target_link_libraries(outline_discovery_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(bounding_box_detection_test
  unit/utils/bounding_box_detection_test.cpp)
target_link_libraries(bounding_box_detection_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(edge_detection_test
  unit/utils/edge_detection_test.cpp)
target_link_libraries(edge_detection_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(hole_filters_test
  unit/utils/hole_filters_test.cpp)
target_link_libraries(hole_filters_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(holes_conveyor_test
  unit/utils/holes_conveyor_test.cpp)
target_link_libraries(holes_conveyor_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(message_conversions_test
  unit/utils/message_conversions_test.cpp)
target_link_libraries(message_conversions_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(morphological_operators_test
  unit/utils/morphological_operators_test.cpp)
target_link_libraries(morphological_operators_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(noise_elimination_test
  unit/utils/noise_elimination_test.cpp)
target_link_libraries(noise_elimination_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(visualization_test
  unit/utils/visualization_test.cpp)
target_link_libraries(visualization_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)

catkin_add_gtest(wavelets_test
  unit/utils/wavelets_test.cpp)
target_link_libraries(wavelets_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main)


################################ depth node ####################################
catkin_add_gtest(depth_hole_detector_test
  unit/depth_node/hole_detector_test.cpp)

target_link_libraries(depth_hole_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_depth
  gtest_main)


################################# rgb node #####################################
catkin_add_gtest(rgb_hole_detector_test
  unit/rgb_node/hole_detector_test.cpp)

target_link_libraries(rgb_hole_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_rgb
  gtest_main)

############################# hole fusion node #################################

###### depth_filters_test.cpp ######
catkin_add_gtest(depth_filters_test
  unit/hole_fusion_node/depth_filters_test.cpp)

target_link_libraries(depth_filters_test
  ${PROJECT_NAME}_filters
  gtest_main)

###### hole_merger_test.cpp ######
catkin_add_gtest(hole_merger_test
  unit/hole_fusion_node/hole_merger_test.cpp)

target_link_libraries(hole_merger_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_filters
  ${PROJECT_NAME}_hole_fusion
  gtest_main)


###### filters_test.cpp ######
catkin_add_gtest(filters_test
  unit/hole_fusion_node/filters_test.cpp)

target_link_libraries(filters_test
 ${PROJECT_NAME}_filters
 gtest_main)


###### filters_resources_test.cpp ######
catkin_add_gtest(filters_resources_test
  unit/hole_fusion_node/filters_resources_test.cpp)

target_link_libraries(filters_resources_test
 ${PROJECT_NAME}_filters
 gtest_main)


###### hole_validation_test.cpp ######
catkin_add_gtest(hole_validation_test
  unit/hole_fusion_node/hole_validation_test.cpp)

target_link_libraries(hole_validation_test
  ${PROJECT_NAME}_hole_fusion
  gtest_main)


###### hole_uniqueness_test.cpp ######
catkin_add_gtest(hole_uniqueness_test
  unit/hole_fusion_node/hole_uniqueness_test.cpp)

target_link_libraries(hole_uniqueness_test
  ${PROJECT_NAME}_hole_fusion
  gtest_main)


###### planes_detection_test.cpp ######
catkin_add_gtest(planes_detection_test
  unit/hole_fusion_node/planes_detection_test.cpp)

target_link_libraries(planes_detection_test
  ${PROJECT_NAME}_filters
  gtest_main)


###### rgb_filters_test.cpp ######
catkin_add_gtest(rgb_filters_test
  unit/hole_fusion_node/rgb_filters_test.cpp)

target_link_libraries(rgb_filters_test
  ${PROJECT_NAME}_filters
  gtest_main)


############## Functional Tests ###############

find_package(rostest REQUIRED)

#catkin_download_test_data(
#    ${PROJECT_NAME}_hole_detector_test.bag
#    http://downloads.pandora.ee.auth.gr/bags/hole_detector_test.bag
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
#    MD5 1844a18c09ee3f7b5725bb788fa43ac1
#    )
#add_rostest(functional/hole_detector_test.launch)
