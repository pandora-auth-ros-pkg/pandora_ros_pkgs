################# MapLoader ############################################

find_package(catkin REQUIRED COMPONENTS
          rostest
          roslib 
          map_server
          )


find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

link_directories(${catkin_LIBRARY_DIRS})
include_directories( test/map_loader/include ${catkin_INCLUDE_DIRS})

add_library(map_loader test/map_loader/src/map_loader.cpp)
target_link_libraries(map_loader 
    image_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

################# Tests ################################################
# add tests here so that CMakelists is not polluted

################### Unit Tests #########################

##########  ObjectListTest ###########  

catkin_add_gtest(object_list_test test/unit/object_list_test.cpp)
target_link_libraries(object_list_test ${catkin_LIBRARIES}  objects utils
  gtest_main) 

##########  ObjectFactoryTest ########### 

catkin_add_gtest(object_factory_test test/unit/object_factory_test.cpp)
target_link_libraries(object_factory_test 
${catkin_LIBRARIES} 
object_factory
map_loader
gtest_main
) 

##########  PoseFinderTest ###########   

set(CMAKE_BUILD_TYPE Debug)
catkin_add_gtest(pose_finder_test test/unit/pose_finder_test.cpp)
target_link_libraries(pose_finder_test ${catkin_LIBRARIES}
    pose_finder map_loader gtest_main)

######### VictimTest #################

catkin_add_gtest(victim_test test/unit/victim_test.cpp)
target_link_libraries(victim_test
  ${catkin_LIBRARIES}
  hole
  tpa
  victim 
  gtest_main
  ) 
   
##########  VictimListTest ###########  

catkin_add_gtest(victim_list_test test/unit/victim_list_test.cpp)
target_link_libraries(victim_list_test 
  ${catkin_LIBRARIES}
  hole
  tpa
  victim_list
  gtest_main
  ) 

######### VictimClustererTest #################

catkin_add_gtest(victim_clusterer_test test/unit/victim_clusterer_test.cpp)
target_link_libraries(victim_clusterer_test ${catkin_LIBRARIES}
  victim_clusterer 
  hole
  tpa
  victim
  gtest_main
  ) 

##########  ObjectsTest #########

catkin_add_gtest(objects_test test/unit/objects_test.cpp)
target_link_libraries(objects_test 
${catkin_LIBRARIES}
  hazmat
  hole
  qr
  tpa
   ) 

########## Functional Tests #########

add_rostest(test/functional/alert_handler_test.launch)

########### RosLint ###################################################

set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include,-whitespace/blank_line,-whitespace/parens, -whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            include/alert_handler/*.h 
            src/*.cpp 
            #test/unit/*.cpp
            )
LIST(REMOVE_ITEM ${PROJECT_NAME}_LINT_SRCS
      RELATIVE ${PROJECT_SOURCE_DIR}
            include/alert_handler/alert_handler.h
            include/alert_handler/tf_listener.h
            src/alert_handler.cpp
            src/ros_tf_listener.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include,-whitespace/blank_line,-whitespace/parens, -whitespace/braces, -runtime/references")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS_WITH_REFS
     RELATIVE ${PROJECT_SOURCE_DIR} 
            include/alert_handler/alert_handler.h
            include/alert_handler/tf_listener.h
            src/alert_handler.cpp
            src/ros_tf_listener.cpp
            )
          roslint_cpp(${${PROJECT_NAME}_LINT_SRCS_WITH_REFS})
roslint_add_test()

