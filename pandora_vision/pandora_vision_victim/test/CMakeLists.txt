find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})


catkin_add_gtest(mean_std_dev_test unit/channels_statistics_feature_extractors/mean_std_dev_test.cpp)
target_link_libraries(mean_std_dev_test
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_channels_statistics_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    gtest_main)

catkin_add_gtest(dominant_color_test unit/channels_statistics_feature_extractors/dominant_color_test.cpp)
target_link_libraries(dominant_color_test
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_channels_statistics_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    gtest_main)

catkin_add_gtest(edge_orientation_extractor_test unit/feature_extractors/edge_orientation_extractor_test.cpp)
target_link_libraries(edge_orientation_extractor_test
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    gtest_main)

catkin_add_gtest(haralickfeature_extractor_test unit/feature_extractors/haralickfeature_extractor_test.cpp)
target_link_libraries(haralickfeature_extractor_test
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    gtest_main)

download_extra_data(
  ${PROJECT_NAME}_sift_test_image
  EXTRA_FILES data/test_data.yaml)

catkin_add_gtest(sift_feature_extractor_test 
  unit/feature_extractors/sift_extractor_test.cpp)
target_link_libraries(sift_feature_extractor_test
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    gtest_main)

catkin_add_gtest(color_histogram_feature_extractor_test
  unit/feature_extractors/histogram_extractor_test.cpp)
target_link_libraries(color_histogram_feature_extractor_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_feature_extractors
  ${PROJECT_NAME}_victim_parameters
  gtest_main)

#catkin_add_gtest(color_angles_test unit/channels_statistics_feature_extractors/color_angles_test.cpp)
#target_link_libraries(color_angles_test ${catkin_LIBRARIES} ${PROJECT_NAME}_channels_statistics_feature_extractors  ${PROJECT_NAME}_victim_parameters gtest_main)

############## Functional Tests ###############

#if(${PROJECT_NAME}_functional)
  #catkin_download_test_data(
    #hole_detector_test.bag
    #https://pandora.ee.auth.gr/rosbags/2014_bags/hole_detector_test.bag
    #DESTINATION functional
    #)
  #add_rostest(functional/hole_detector_test.launch)

  #add_rostest(functional/victim_test.launch)
#endif()

#################################################################################
##                               Benchmark Tests                                #
#################################################################################

#if(${PROJECT_NAME}_benchmark)
  #download_extra_data(
    #${PROJECT_NAME}_benchmark_images_bags_pcl
    #EXTRA_FILES test/benchmark/images/bags_pcl/extra_files.yaml
    ##EXCLUDE_FROM_ALL
    #)
  #add_dependencies(${PROJECT_NAME}_benchmark_images_bags_pcl tests)

  #add_rostest(benchmark/victim_benchmark_test.launch DEPENDENCIES
            #${PROJECT_NAME}_benchmark_images_bags_pcl)
#endif()
