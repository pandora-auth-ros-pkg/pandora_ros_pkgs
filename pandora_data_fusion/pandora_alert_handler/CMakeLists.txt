cmake_minimum_required(VERSION 2.8.3)
project(pandora_alert_handler)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf 
  actionlib 
  dynamic_reconfigure
  roslint
  pandora_data_fusion_msgs 
  pandora_common_msgs
  vision_communications
  )

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

generate_dynamic_reconfigure_options(
  config/alert_handler/AlertHandler.cfg
  config/alert_handler/mass_alert_publisher_dymamic.cfg
  )

# Cmake does not support Coverage option by default, 
# so we need to do it manually. 
# To compile with coverage enabled you should invoke `catkin_make` 
# with `-DCMAKE_BUILD_TYPE=Coverage`.
# NOTE: you should not set `CMAKE_BUILD_TYPE` variable explicitly
# in CMakeLists.txt.
set(CMAKE_CXX_FLAGS_COVERAGE "-fprofile-arcs -ftest-coverage")

catkin_package(
  DEPENDS
    orocos-bfl
  CATKIN_DEPENDS 
    roscpp
    tf 
    actionlib 
    dynamic_reconfigure
    roslint
    pandora_data_fusion_msgs 
    pandora_common_msgs
    vision_communications
  INCLUDE_DIRS 
    include
    ${BFL_INCLUDE_DIRS}
  LIBRARIES 
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
  )
link_directories(
  ${BFL_LIBRARY_DIRS}
  )
          
########################  utils  ####################################

add_library(utils 
  src/utils.cpp)
target_link_libraries(utils 
  ${catkin_LIBRARIES}
  )

########################  filter_model  #############################

add_library(filter_model
  src/filter_model.cpp)
target_link_libraries(filter_model 
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
  )

########################  objects  ##################################

add_library(base_object
  src/base_object.cpp)
target_link_libraries(base_object
  ${catkin_LIBRARIES}
  )
add_dependencies(base_object 
  pandora_data_fusion_msgs_gencpp
  ) 

add_library(objects
  src/hole.cpp
  src/hazmat.cpp
  src/landoltc.cpp
  src/data_matrix.cpp
  src/qr.cpp
  src/thermal.cpp
  src/face.cpp
  src/motion.cpp
  src/sound.cpp
  src/co2.cpp
  )
target_link_libraries(objects
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
  utils
  base_object
  filter_model
  )

add_library(victim 
  src/victim.cpp)
target_link_libraries(victim
  ${catkin_LIBRARIES}
  utils
  base_object
  objects
  )

########################  victim_list  ##############################

add_library(victim_list 
  src/victim_list.cpp)
target_link_libraries(victim_list
  ${catkin_LIBRARIES}
  victim
  )

########################  ros_tf_listener  ##########################

add_library(ros_tf_listener 
  src/ros_tf_listener.cpp)
target_link_libraries(ros_tf_listener 
  ${catkin_LIBRARIES}
  )

########################  pose_finder  ##############################

add_library(pose_finder 
  src/pose_finder.cpp)
target_link_libraries(pose_finder
  ${catkin_LIBRARIES}
  utils
  ros_tf_listener
  )

########################  object_factory  ###########################

add_library(object_factory 
  src/object_factory.cpp)
target_link_libraries(object_factory
  ${catkin_LIBRARIES}
  pose_finder
  objects
  )
add_dependencies(object_factory 
  pandora_data_fusion_msgs_gencpp
  vision_communications_gencpp
  pandora_common_msgs_gencpp
  ) 
         
########################  victim_clusterer  #########################

add_library(victim_clusterer 
  src/victim_clusterer.cpp)
target_link_libraries(victim_clusterer 
  ${catkin_LIBRARIES}
  utils
  victim
  )

########################  victim_handler  ###########################

add_library(victim_handler 
  src/victim_handler.cpp)
target_link_libraries(victim_handler
  ${catkin_LIBRARIES}
  utils
  victim
  victim_list
  victim_clusterer
  )
          
########################  object_handler  ###########################

add_library(object_handler 
  src/object_handler.cpp)
target_link_libraries(object_handler 
  ${catkin_LIBRARIES}
  utils
  objects
  victim_list
  )
add_dependencies(object_handler 
  pandora_data_fusion_msgs_gencpp
  ) 

########################  alert_handler  ############################

add_library(alert_handler 
  src/alert_handler.cpp)
target_link_libraries(alert_handler 
  ${catkin_LIBRARIES}
  object_factory
  object_handler
  victim_handler
  victim_list
  )
add_dependencies(alert_handler 
  pandora_alert_handler_gencfg 
  vision_communications_gencpp
  pandora_data_fusion_msgs_gencpp
  pandora_common_msgs_gencpp
  ) 

########################  alert_handler_node  #######################

add_executable(alert_handler_node 
  src/main.cpp )
target_link_libraries(alert_handler_node 
  ${catkin_LIBRARIES}
  alert_handler
  )

########################  roslint  ##################################

set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include,-whitespace/blank_line,-whitespace/parens, -whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
    include/alert_handler/*.h 
    src/*.cpp
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
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include,-whitespace/blank_line,-whitespace/parens,-whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_TEST 
  RELATIVE ${PROJECT_SOURCE_DIR} 
    test/unit/*
    )
roslint_cpp(${${PROJECT_NAME}_LINT_TEST})

########################  testing  ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

