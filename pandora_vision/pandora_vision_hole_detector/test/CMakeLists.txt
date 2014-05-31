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

catkin_add_gtest(blob_detection_test
  unit/utils/blob_detection_test.cpp)
target_link_libraries(blob_detection_test ${catkin_LIBRARIES} utils_lib gtest_main)

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

catkin_add_gtest(morphological_operators_test
  unit/utils/morphological_operators_test.cpp)
target_link_libraries(morphological_operators_test ${catkin_LIBRARIES} utils_lib gtest_main)

catkin_add_gtest(noise_elimination_test
  unit/utils/noise_elimination_test.cpp)
target_link_libraries(noise_elimination_test ${catkin_LIBRARIES} utils_lib gtest_main)


################################ depth node ####################################
catkin_add_gtest(hole_detector_test
  unit/depth_node/hole_detector_test.cpp)

add_library(depth_node_lib
  ../src/depth_node/hole_detector.cpp)

target_link_libraries(hole_detector_test ${catkin_LIBRARIES} utils_lib depth_node_lib gtest_main)

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


###### filters_resources_test.cpp ######
catkin_add_gtest(filters_resources_test
  unit/hole_fusion_node/filters_resources_test.cpp)

add_library(filters_resources_lib
  ../src/hole_fusion_node/filters_resources.cpp)

target_link_libraries(filters_resources_test ${catkin_LIBRARIES} utils_lib filters_resources_lib gtest_main)

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

target_link_libraries(rgb_filters_test ${catkin_LIBRARIES} utils_lib rgb_filters_lib gtest_main)


############## Functional Tests ###############

#if(TARGET tests)
#  add_executable(hole_detector_test EXCLUDE_FROM_ALL functional/hole_detector_test.cpp)
#  target_link_libraries(hole_detector_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} vision_communications_gencpp)
#endif()
#add_rostest_gtest(hole_detector_test functional/hole_detector_test.launch functional/hole_detector_test.cpp)
#target_link_libraries(hole_detector_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#add_dependencies(hole_detector_test vision_communications_gencpp node_tests node_tests_msgs_gencpp)
#add_rostest(functional/hole_detector_test.launch)
