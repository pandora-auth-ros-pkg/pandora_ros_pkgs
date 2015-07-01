cmake_minimum_required(VERSION 2.8.3)
project(pandora_data_fusion_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roslint
  geometry_msgs
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
    geometry_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    tf_listener_wrapper
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

################################################################################
#                          pandora_data_fusion_utils                           #
################################################################################

## Utils
add_library(${PROJECT_NAME}
  src/pandora_data_fusion_utils/utils.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

## RosTfListener
add_library(tf_listener_wrapper
  src/pandora_data_fusion_utils/ros_tf_listener.cpp
  )
target_link_libraries(tf_listener_wrapper
  ${catkin_LIBRARIES}
  )

########################  roslint  ##################################

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/**/*.h
    src/**/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

########################  testing  ##################################

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()

##################### Install targets ###############################

### Install headers
install(DIRECTORY include/pandora_data_fusion_utils/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

## Install exported libraries
install(TARGETS ${PROJECT_NAME}
                tf_listener_wrapper
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
