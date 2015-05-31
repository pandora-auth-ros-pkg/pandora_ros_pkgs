find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})

################################################################################
#                               Unit Tests                                     #
################################################################################

# catkin_add_gtest(${PROJECT_NAME}_2d_detector_test
  # unit/landoltc_2d/landoltc_detector_test.cpp
# )
# target_link_libraries(${PROJECT_NAME}_2d_detector_test
  # ${catkin_LIBRARIES}
  # ${PROJECT_NAME}_2d_detector
  # gtest_main
# )

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

  add_rostest(benchmark/landoltc_benchmark_test.launch
              ARGS xtion:=true)
  add_rostest(benchmark/landoltc_benchmark_test.launch
              ARGS usb_cam:=true)

endif()
