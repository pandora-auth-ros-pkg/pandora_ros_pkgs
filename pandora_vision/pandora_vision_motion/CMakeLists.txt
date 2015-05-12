cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_motion)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_common_msgs
    pandora_vision_common
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    roslint
    urdf
)


generate_dynamic_reconfigure_options(
  config/motion_cfg.cfg
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_common_msgs
      pandora_vision_common
      state_manager
      state_manager_msgs
      cv_bridge
      image_transport
      sensor_msgs
      dynamic_reconfigure
      urdf
    INCLUDE_DIRS
      include
    LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################# motion detector #################
add_library(${PROJECT_NAME}_detector
  src/motion_detector.cpp
  src/motion_parameters.cpp
)
add_dependencies(${PROJECT_NAME}_detector
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_detector
  ${catkin_LIBRARIES}
)

################# motion preprocessor #################
add_library(${PROJECT_NAME}_preprocessor
  src/motion_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_preprocessor
  ${catkin_LIBRARIES}
)

################# motion postprocessor #################
add_library(${PROJECT_NAME}_postprocessor
  src/motion_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_postprocessor
  ${catkin_LIBRARIES}
)

################# motion handler #################
add_library(${PROJECT_NAME}_handler
  src/motion_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_postprocessor
)

################# motion node #################
add_executable(${PROJECT_NAME}_node
    src/motion_node.cpp
    # src/dbscan.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_handler
)
add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#~ To check lint : catkin_make roslint_pandora_vision_motion
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line,-build/include_what_you_use,-runtime/references")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/*
  src/*
  test/*
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

################# Testing #################
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
