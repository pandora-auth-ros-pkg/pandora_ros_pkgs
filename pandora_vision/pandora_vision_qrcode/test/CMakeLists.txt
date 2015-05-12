find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
${pandora_testing_tools_INCLUDE_DIRS}
${roslib_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS})

################################################################################
#                               Functional Tests                               #
################################################################################

# download_extra_data(
  # ${PROJECT_NAME}_qr_test_images
  # EXTRA_FILES test/functional/images/extra_files.yaml
  # )
# add_dependencies(${PROJECT_NAME}_qr_test_images tests)

# add_rostest(functional/qr_test.launch)


################# Unit Tests #################

catkin_add_gtest(${PROJECT_NAME}_detector_test
  unit/qrCode_detector_test.cpp
)
target_link_libraries(${PROJECT_NAME}_detector_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  gtest_main
)

download_extra_data(
    ${PROJECT_NAME}_download_qr_images_unit_testing
    EXTRA_FILES test/unit/data/extra_files.yaml
    )

################################################################################
#                               Benchmark Tests                                #
################################################################################

#download_extra_data(
  #${PROJECT_NAME}_benchmark_images_480p
  #EXTRA_FILES test/benchmark/images/images_480p/extra_files.yaml
  ##EXCLUDE_FROM_ALL
  #)
#add_dependencies(${PROJECT_NAME}_benchmark_images_480p tests)

#download_extra_data(
  #${PROJECT_NAME}_benchmark_images_720p
  #EXTRA_FILES test/benchmark/images/images_720p/extra_files.yaml
  ##EXCLUDE_FROM_ALL
  #)
#add_dependencies(${PROJECT_NAME}_benchmark_images_720p tests)

#add_rostest(benchmark/qrcode_kinect_benchmark_test.launch)
#add_rostest(benchmark/qrcode_web_camera_benchmark_test.launch)

# add_rostest(benchmark/qrcode_kinect_benchmark_test.launch DEPENDENCIES ${PROJECT_NAME}_benchmark_images_480p)
# add_rostest(benchmark/qrcode_web_camera_benchmark_test.launch DEPENDENCIES ${PROJECT_NAME}_benchmark_images_720p)

