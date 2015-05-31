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
#                             Download Benchmark Data                        #
##############################################################################
if (${PROJECT_NAME}_download)

  # Download the images for the kinect/xtion benchmarking.
  download_extra_data(
    ${PROJECT_NAME}_benchmark_images_480p
    EXTRA_FILES test/benchmark/images/images_480p/extra_files.yaml
    )
  # Download the images for the 720p web cam benchmarking.
  download_extra_data(
    ${PROJECT_NAME}_benchmark_images_720p
    EXTRA_FILES test/benchmark/images/images_720p/extra_files.yaml
    )
endif()
################################################################################
#                               Benchmark Tests                                #
################################################################################

if (${PROJECT_NAME}_benchmark)

  add_rostest(benchmark/hazmat_benchmark_test.launch
    ARGS xtion:=true)
  add_rostest(benchmark/hazmat_benchmark_test.launch
    ARGS usb_cam:=true)

endif()
