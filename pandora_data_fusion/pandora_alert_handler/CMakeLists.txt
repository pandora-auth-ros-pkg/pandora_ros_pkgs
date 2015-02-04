cmake_minimum_required(VERSION 2.8.3)
project(pandora_alert_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  dynamic_reconfigure
  roslint
  visualization_msgs
  pandora_data_fusion_msgs
  pandora_common_msgs
  pandora_vision_msgs
  )

# http://wiki.ros.org/bfl
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)

generate_dynamic_reconfigure_options(
  config/alert_handler/AlertHandler.cfg
  config/alert_handler/MassAlertPublisher.cfg
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
    pandora_data_fusion_msgs
    pandora_common_msgs
    pandora_vision_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_ros_tf_listener
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

add_library(${PROJECT_NAME}_utils
  src/utils.cpp)
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )

########################  filter_model  #############################

add_library(${PROJECT_NAME}_filter_model
  src/filter_model.cpp)
target_link_libraries(${PROJECT_NAME}_filter_model
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
  )

########################  objects  ##################################

add_library(${PROJECT_NAME}_base_object
  src/base_object.cpp)
target_link_libraries(${PROJECT_NAME}_base_object
  ${catkin_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}_base_object
  ${catkin_EXPORTED_TARGETS}
  )

add_library(${PROJECT_NAME}_objects
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
target_link_libraries(${PROJECT_NAME}_objects
  ${catkin_LIBRARIES}
  ${BFL_LIBRARIES}
  ${PROJECT_NAME}_base_object
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_filter_model
  )

add_library(${PROJECT_NAME}_victim
  src/victim.cpp)
target_link_libraries(${PROJECT_NAME}_victim
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_objects
  )

########################  victim_list  ##############################

add_library(${PROJECT_NAME}_victim_list
  src/victim_list.cpp)
target_link_libraries(${PROJECT_NAME}_victim_list
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim
  )

########################  ros_tf_listener  ##########################

add_library(${PROJECT_NAME}_ros_tf_listener
  src/ros_tf_listener.cpp)
target_link_libraries(${PROJECT_NAME}_ros_tf_listener
  ${catkin_LIBRARIES}
  )

########################  pose_finder  ##############################

add_library(${PROJECT_NAME}_pose_finder
  src/pose_finder.cpp)
target_link_libraries(${PROJECT_NAME}_pose_finder
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_ros_tf_listener
  ${PROJECT_NAME}_base_object
  )

########################  object_factory  ###########################

add_library(${PROJECT_NAME}_object_factory
  src/object_factory.cpp)
target_link_libraries(${PROJECT_NAME}_object_factory
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_pose_finder
  ${PROJECT_NAME}_objects
  )
add_dependencies(${PROJECT_NAME}_object_factory ${catkin_EXPORTED_TARGETS})

########################  victim_clusterer  #########################

add_library(${PROJECT_NAME}_victim_clusterer
  src/victim_clusterer.cpp)
target_link_libraries(${PROJECT_NAME}_victim_clusterer
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_victim
  )

########################  victim_handler  ###########################

add_library(${PROJECT_NAME}_victim_handler
  src/victim_handler.cpp)
target_link_libraries(${PROJECT_NAME}_victim_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_victim
  ${PROJECT_NAME}_victim_list
  ${PROJECT_NAME}_victim_clusterer
  )

########################  object_handler  ###########################

add_library(${PROJECT_NAME}_object_handler
  src/object_handler.cpp)
target_link_libraries(${PROJECT_NAME}_object_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_objects
  ${PROJECT_NAME}_victim_list
  )
add_dependencies(${PROJECT_NAME}_object_handler ${catkin_EXPORTED_TARGETS})

########################  alert_handler  ############################

add_library(${PROJECT_NAME}_alert_handler
  src/alert_handler.cpp)
target_link_libraries(${PROJECT_NAME}_alert_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_object_factory
  ${PROJECT_NAME}_object_handler
  ${PROJECT_NAME}_victim_handler
  ${PROJECT_NAME}_victim_list
  )
add_dependencies(${PROJECT_NAME}_alert_handler
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

########################  alert_handler_node  #######################

add_executable(alert_handler_node
  src/main.cpp )
target_link_libraries(alert_handler_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_alert_handler
  )

########################  roslint  ##################################

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/alert_handler/*.h
    src/*.cpp
    )
list(REMOVE_ITEM ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/alert_handler/alert_handler.h
    include/alert_handler/tf_listener.h
    src/alert_handler.cpp
    src/ros_tf_listener.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
set(ROSLINT_CPP_OPTS
    "--filter=-runtime/references")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS_WITH_REFS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/alert_handler/alert_handler.h
    include/alert_handler/tf_listener.h
    src/alert_handler.cpp
    src/ros_tf_listener.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS_WITH_REFS})
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_TEST
  RELATIVE ${PROJECT_SOURCE_DIR}
    test/unit/*
    )
roslint_cpp(${${PROJECT_NAME}_LINT_TEST})

########################  testing  ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()



##################### Install targets ###############################

### Install headers
install(DIRECTORY include/alert_handler/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Install exported libraries
install(TARGETS ${PROJECT_NAME}_utils
                ${PROJECT_NAME}_ros_tf_listener
                ${PROJECT_NAME}_alert_handler
                ${PROJECT_NAME}_victim_list
                ${PROJECT_NAME}_victim_handler
                ${PROJECT_NAME}_object_handler
                ${PROJECT_NAME}_objects
                ${PROJECT_NAME}_base_object
                ${PROJECT_NAME}_filter_model
                ${PROJECT_NAME}_pose_finder
                ${PROJECT_NAME}_object_factory
                ${PROJECT_NAME}_victim
                ${PROJECT_NAME}_victim_clusterer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Install executables
install(TARGETS alert_handler_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
