find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

################################################################################
#                               Benchmark Tests                                #
################################################################################

# add_rostest(benchmark/hazmat_benchmark_test.launch)

include_directories(
  ${pandora_testing_tools_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

catkin_add_gtest(feature_detector_test unit/feature_matching_test.cpp)
target_link_libraries(feature_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sift_detector
  ${PROJECT_NAME}_surf_detector
  ${PROJECT_NAME}_orb_detector
  gtest_main
  gtest
  ${GTEST_LIBRARIES} 
  )

catkin_add_gtest(image_signature_test unit/image_signature_test.cpp)
target_link_libraries(image_signature_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_filters
  gtest_main
  gtest
  )

##############################################################################
#                               Functional Tests                             #
##############################################################################

#if(${PROJECT_NAME}_functional)
  #download_extra_data(
    #${PROJECT_NAME}_hazmat_test_images
    #EXTRA_FILES test/functional/images/extra_files.yaml
    #)
  #add_dependencies(${PROJECT_NAME}_hazmat_test_images tests)

  #add_rostest(functional/hazmat_test.launch
    #DEPENDENCIES ${PROJECT_NAME}_hazmat_test_images)
#endif()

################################################################################
#                               Benchmark Tests                                #
################################################################################

#if (${PROJECT_NAME}_benchmark)

  ## Download the images for the kinect/xtion benchmarking.
  #download_extra_data(
    #${PROJECT_NAME}_benchmark_images_480p
    #EXTRA_FILES test/benchmark/images/images_480p/extra_files.yaml
    #EXCLUDE_FROM_ALL
    #)
  #add_dependencies(${PROJECT_NAME}_benchmark_images_480p tests)

  ## Download the images for the high definition web images.
  #download_extra_data(
    #${PROJECT_NAME}_benchmark_images_720p
    #EXTRA_FILES test/benchmark/images/images_720p/extra_files.yaml
    #EXCLUDE_FROM_ALL
    #)
  #add_dependencies(${PROJECT_NAME}_benchmark_images_720p tests)

  #add_rostest(benchmark/landoltc_kinect_benchmark_test.launch
    #DEPENDENCIES ${PROJECT_NAME}_benchmark_images_480p)
  #add_rostest(benchmark/landoltc_web_camera_benchmark_test.launch
    #DEPENDENCIES ${PROJECT_NAME}_benchmark_images_720p)

#endif()
