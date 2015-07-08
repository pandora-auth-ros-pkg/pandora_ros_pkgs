################################################################################
#                                 Find Package                                 #
################################################################################
# - Use `find_package` command to bring dependencies. Don't override catkin
#   dependency group. This is declared by package's main CMakeLists.txt
# - Set flags if needed
# - Reset `include_directories`. Don't forget to list again the already set
#   ${catkin_INCLUDE_DIRS} together with those you will be newly declaring

find_package(roslib REQUIRED)
find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)
find_package(pandora_cmake_tools REQUIRED)

include_directories(
  ${pandora_testing_tools_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

################################################################################
#                                    Tests                                     #
################################################################################

################################################################################
#                              Download Test Data                              #
################################################################################
# - Use `catkin_download_test_data` to download a specific file from the web
#   which is to be used for testing
# - Use `download_extra_data` to download a testing data base from the web
#   according to an `EXTRA_FILES` yaml file
# - Prefer `${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test` as
#   a testing data `DESTINATION`, so as not to pollute catkin's src directory
#   with extra data
# - DON'N forget to `find_package(pandora_cmake_tools REQUIRED)` if you intend
#   to use `download_extra_data`
download_data(
  ${PROJECT_NAME}_frame_matcher_test.bag
  http://downloads.pandora.ee.auth.gr/bags/frame_matcher_test.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test/functional
  MD5 09bd8ca9d31ba8da19fd2449256bc9d7
  )

################################################################################
#                                  Unit Tests                                  #
################################################################################
# - Use `catkin_add_gtest` to declare test targets for cpp libraries
# - Use `catkin_add_nosetests` to declare test targets for python libraries
# - All unit tests' code should be listed under ./unit subdirectory
# - Set `target_link_libraries` for each test target
# - Set `add_dependencies`, if necessary,
#   to ${catkin_EXPORTED_TARGETS} or others
# - For gtests you can add `gtest_main` as a target link library to enable its
#   executation when `catkin_make run_tests` is invoked

################################################################################
#                               Functional Tests                               #
################################################################################
# - Use `add_rostest` to declare that a functional test launch file will be
#   executed when `catkin_make run_tests` is invoked. Convenient for functional
#   tests written in python
# - Use 'add_rostest_gtest` to declare a functional test which is made from
#   gtest cpp code by `add_rostest_gtest(my_gtest_rostest_target
#   functional/launch/my_gtest_rostest.launch
#   functional/src/my_gtest_rostest.cpp [more cpp files])`
# - Set `target_link_libraries` for each functional test target. By ros indigo,
#   you can use `DEPENDENCIES` to declare a rostest target dependency
# - DON'T forget to `find_package(rostest REQUIRED)` if you intend to do
#   functional testing

add_rostest(functional/launch/frame_matcher_test.launch)

################################################################################
#                                   Roslint
################################################################################
# - Setup roslint for test code with `roslint_cpp`, `roslint_python` or
#   `roslint_custom`
# - You can call `roslint_add_test` also
