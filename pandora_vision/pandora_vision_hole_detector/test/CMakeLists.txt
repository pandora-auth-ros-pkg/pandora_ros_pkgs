find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  #node_tests
  #node_tests_msgs
  rostest
  roslint
  )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###############################  Unit Tests ####################################

#################################### utils #####################################

catkin_add_gtest(outline_discovery_test
  unit/utils/outline_discovery_test.cpp)
target_link_libraries(outline_discovery_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(bounding_box_detection_test
  unit/utils/bounding_box_detection_test.cpp)
target_link_libraries(bounding_box_detection_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(edge_detection_test
  unit/utils/edge_detection_test.cpp)
target_link_libraries(edge_detection_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(hole_filters_test
  unit/utils/hole_filters_test.cpp)
target_link_libraries(hole_filters_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(holes_conveyor_test
  unit/utils/holes_conveyor_test.cpp)
target_link_libraries(holes_conveyor_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(message_conversions_test
  unit/utils/message_conversions_test.cpp)
target_link_libraries(message_conversions_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(morphological_operators_test
  unit/utils/morphological_operators_test.cpp)
target_link_libraries(morphological_operators_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(noise_elimination_test
  unit/utils/noise_elimination_test.cpp)
target_link_libraries(noise_elimination_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(visualization_test
  unit/utils/visualization_test.cpp)
target_link_libraries(visualization_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(wavelets_test
  unit/utils/wavelets_test.cpp)
target_link_libraries(wavelets_test ${catkin_LIBRARIES} utils_lib gtest_main)


################################ depth node ####################################
catkin_add_gtest(depth_hole_detector_test
  unit/depth_node/hole_detector_test.cpp)

add_library(depth_node_lib
  ../src/depth_node/hole_detector.cpp)

target_link_libraries(depth_hole_detector_test ${catkin_LIBRARIES} depth_node_lib utils_lib gtest_main)


################################# rgb node #####################################
catkin_add_gtest(rgb_hole_detector_test
  unit/rgb_node/hole_detector_test.cpp)

add_library(rgb_node_lib
  ../src/rgb_node/hole_detector.cpp)

target_link_libraries(rgb_hole_detector_test ${catkin_LIBRARIES} rgb_node_lib utils_lib gtest_main)

############################# hole fusion node #################################

###### depth_filters_test.cpp ######
catkin_add_gtest(depth_filters_test
  unit/hole_fusion_node/depth_filters_test.cpp)


add_library(depth_filters_lib
  ../src/hole_fusion_node/depth_filters.cpp
  ../src/hole_fusion_node/filters_resources.cpp
  ../src/hole_fusion_node/planes_detection.cpp
  )

target_link_libraries(depth_filters_test depth_filters_lib pcl_filters ${PCL_LIBRARIES} utils_lib gtest_main)


###### hole_merger_test.cpp ######
catkin_add_gtest(hole_merger_test
  unit/hole_fusion_node/hole_merger_test.cpp)

add_library(hole_merger_lib
  ../src/hole_fusion_node/depth_filters.cpp
  ../src/hole_fusion_node/filters_resources.cpp
  ../src/hole_fusion_node/hole_merger.cpp
  ../src/hole_fusion_node/planes_detection.cpp
  )

target_link_libraries(hole_merger_test hole_merger_lib pcl_filters ${PCL_LIBRARIES} ${catkin_LIBRARIES} utils_lib gtest_main)

###### filters_test.cpp ######
catkin_add_gtest(filters_test
  unit/hole_fusion_node/filters_test.cpp)


add_library(filters_lib
  ../src/hole_fusion_node/depth_filters.cpp
  ../src/hole_fusion_node/filters.cpp
  ../src/hole_fusion_node/filters_resources.cpp
  ../src/hole_fusion_node/planes_detection.cpp
  ../src/hole_fusion_node/rgb_filters.cpp)

target_link_libraries(filters_test filters_lib pcl_filters ${PCL_LIBRARIES} utils_lib gtest_main)



###### filters_resources_test.cpp ######
catkin_add_gtest(filters_resources_test
  unit/hole_fusion_node/filters_resources_test.cpp)

add_library(filters_resources_lib
  ../src/hole_fusion_node/filters_resources.cpp)

target_link_libraries(filters_resources_test ${catkin_LIBRARIES} filters_resources_lib utils_lib gtest_main)

###### planes_detection_test.cpp ######
catkin_add_gtest(planes_detection_test
  unit/hole_fusion_node/planes_detection_test.cpp)

add_library(planes_detection_lib
  ../src/hole_fusion_node/planes_detection.cpp)

target_link_libraries(planes_detection_test planes_detection_lib pcl_filters ${PCL_LIBRARIES} utils_lib gtest_main)

###### rgb_filters_test.cpp ######
catkin_add_gtest(rgb_filters_test
  unit/hole_fusion_node/rgb_filters_test.cpp)

add_library(rgb_filters_lib
  ../src/hole_fusion_node/filters_resources.cpp
  ../src/hole_fusion_node/rgb_filters.cpp)

target_link_libraries(rgb_filters_test ${catkin_LIBRARIES} rgb_filters_lib utils_lib gtest_main)


############## Functional Tests ###############

#catkin_download_test_data(
#    hole_detector_test.bag
#    https://pandora.ee.auth.gr/rosbags/2014_bags/hole_detector_test.bag
#    DESTINATION functional
#    )
#add_rostest(functional/hole_detector_test.launch)
