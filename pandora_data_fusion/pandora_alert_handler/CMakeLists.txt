cmake_minimum_required(VERSION 2.8.3)
project(pandora_alert_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  dynamic_reconfigure
  roslint
  visualization_msgs
  geometry_msgs
  nav_msgs
  pandora_data_fusion_msgs
  pandora_common_msgs
  pandora_vision_msgs
  pose_finder
  pandora_data_fusion_utils
  )

# http://wiki.ros.org/bfl
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)

generate_dynamic_reconfigure_options(
  config/pandora_alert_handler/AlertHandler.cfg
  config/pandora_alert_handler/MassAlertPublisher.cfg
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
    bfl
  CATKIN_DEPENDS
    roscpp
    tf
    actionlib
    dynamic_reconfigure
    visualization_msgs
    geometry_msgs
    nav_msgs
    pandora_data_fusion_msgs
    pandora_common_msgs
    pandora_vision_msgs
    pose_finder
    pandora_data_fusion_utils
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_filter_model
    ${PROJECT_NAME}_objects
    ${PROJECT_NAME}_victim
    ${PROJECT_NAME}_obstacle_list
    ${PROJECT_NAME}_victim_list
    ${PROJECT_NAME}_object_factory
    ${PROJECT_NAME}_object_handler
    ${PROJECT_NAME}_victim_clusterer
    ${PROJECT_NAME}_victim_handler
    ${PROJECT_NAME}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
  )
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${BFL_LIBRARY_DIRS}
  )

################################################################################
#                            pandora_alert_handler                             #
################################################################################


## FilterModel
add_library(${PROJECT_NAME}_filter_model
  src/pandora_alert_handler/objects/object_interface/filter_model.cpp
  )
target_link_libraries(${PROJECT_NAME}_filter_model
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
  )

## Objects
add_library(${PROJECT_NAME}_objects
  src/pandora_alert_handler/objects/hole.cpp
  src/pandora_alert_handler/objects/hazmat.cpp
  src/pandora_alert_handler/objects/landoltc.cpp
  src/pandora_alert_handler/objects/data_matrix.cpp
  src/pandora_alert_handler/objects/qr.cpp
  src/pandora_alert_handler/objects/thermal.cpp
  src/pandora_alert_handler/objects/visual_victim.cpp
  src/pandora_alert_handler/objects/motion.cpp
  src/pandora_alert_handler/objects/sound.cpp
  src/pandora_alert_handler/objects/co2.cpp
  src/pandora_alert_handler/objects/obstacle.cpp
  src/pandora_alert_handler/objects/barrel.cpp
  src/pandora_alert_handler/objects/soft_obstacle.cpp
  src/pandora_alert_handler/objects/hard_obstacle.cpp
  )
target_link_libraries(${PROJECT_NAME}_objects
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
  ${PROJECT_NAME}_filter_model
  )
add_dependencies(${PROJECT_NAME}_objects
  ${catkin_EXPORTED_TARGETS}
  )

## ObstacleList
add_library(${PROJECT_NAME}_obstacle_list
  src/pandora_alert_handler/object_lists/obstacle_list.cpp
  )
target_link_libraries(${PROJECT_NAME}_obstacle_list
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  )

## Victim
add_library(${PROJECT_NAME}_victim
  src/pandora_alert_handler/objects/victim.cpp
  )
target_link_libraries(${PROJECT_NAME}_victim
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  )

## VictimList
add_library(${PROJECT_NAME}_victim_list
  src/pandora_alert_handler/object_lists/victim_list.cpp
  )
target_link_libraries(${PROJECT_NAME}_victim_list
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim
  )

## ObjectFactory
add_library(${PROJECT_NAME}_object_factory
  src/pandora_alert_handler/object_factory.cpp
  )
target_link_libraries(${PROJECT_NAME}_object_factory
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  )
add_dependencies(${PROJECT_NAME}_object_factory
  ${catkin_EXPORTED_TARGETS}
  )

## ObjectHandler
add_library(${PROJECT_NAME}_object_handler
  src/pandora_alert_handler/handlers/object_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_object_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_obstacle_list
  ${PROJECT_NAME}_victim_list
  )
add_dependencies(${PROJECT_NAME}_object_handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

## VictimClusterer
add_library(${PROJECT_NAME}_victim_clusterer
  src/pandora_alert_handler/handlers/victim_clusterer.cpp
  )
target_link_libraries(${PROJECT_NAME}_victim_clusterer
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim
  )

## VictimHandler
add_library(${PROJECT_NAME}_victim_handler
  src/pandora_alert_handler/handlers/victim_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_victim_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim
  ${PROJECT_NAME}_victim_list
  ${PROJECT_NAME}_victim_clusterer
  )
add_dependencies(${PROJECT_NAME}_victim_handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

## AlertHandler
add_library(${PROJECT_NAME}
  src/pandora_alert_handler/alert_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_object_factory
  ${PROJECT_NAME}_object_handler
  ${PROJECT_NAME}_victim_handler
  ${PROJECT_NAME}_obstacle_list
  ${PROJECT_NAME}_victim_list
  )
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

## Main
add_executable(${PROJECT_NAME}_node
  src/pandora_alert_handler_node.cpp
  )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

########################  roslint  ##################################

# set(ROSLINT_CPP_OPTS
#     "--filter=-whitespace/braces")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/**/*.h
    src/**/*.cpp
    )
list(REMOVE_ITEM ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/pandora_alert_handler/alert_handler.h
    include/pandora_data_fusion_utils/tf_listener.h
    src/pandora_alert_handler/alert_handler.cpp
    src/pandora_data_fusion_utils/ros_tf_listener.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
set(ROSLINT_CPP_OPTS
    "--filter=-runtime/references")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS_WITH_REFS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/pandora_alert_handler/alert_handler.h
    include/pandora_data_fusion_utils/tf_listener.h
    src/pandora_alert_handler/alert_handler.cpp
    src/pandora_data_fusion_utils/ros_tf_listener.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS_WITH_REFS})
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_TEST_CPP
  RELATIVE ${PROJECT_SOURCE_DIR}
    test/**/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_TEST})
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_TEST_CPP
  RELATIVE ${PROJECT_SOURCE_DIR}
    test/**/*.py
    )
roslint_python(${${PROJECT_NAME}_LINT_TEST})

########################  testing  ##################################

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()

##################### Install targets ###############################

### Install headers
install(DIRECTORY include/pandora_alert_handler/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

## Install exported libraries
install(TARGETS ${PROJECT_NAME}_filter_model
                ${PROJECT_NAME}_objects
                ${PROJECT_NAME}_victim
                ${PROJECT_NAME}_obstacle_list
                ${PROJECT_NAME}_victim_list
                ${PROJECT_NAME}_object_handler
                ${PROJECT_NAME}_object_factory
                ${PROJECT_NAME}_victim_clusterer
                ${PROJECT_NAME}_victim_handler
                ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install executables
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
