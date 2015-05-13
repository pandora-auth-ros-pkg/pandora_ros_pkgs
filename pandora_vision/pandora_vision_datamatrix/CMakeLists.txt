cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_datamatrix)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    state_manager
    state_manager_msgs
    pandora_common_msgs
    pandora_vision_msgs
    pandora_vision_common
    cv_bridge
    image_transport
    sensor_msgs
    roslint
    urdf
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(DMTX REQUIRED)


catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      state_manager
      state_manager_msgs
      pandora_common_msgs
      pandora_vision_msgs
      pandora_vision_common
      cv_bridge
      image_transport
      sensor_msgs
      urdf
    #INCLUDE_DIRS
    #LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${DMTX_INCLUDE_DIRS}
)

################# datamatrix detector #################

add_library(${PROJECT_NAME}_detector
  src/datamatrix_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_detector
  ${catkin_LIBRARIES}
  ${DMTX_LIBRARIES}
)

################# datamatrix preprocessor #################
add_library(${PROJECT_NAME}_preprocessor
  src/datamatrix_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_preprocessor
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_preprocessor
  ${catkin_EXPORTED_TARGETS})

################# datamatrix postprocessor #################
add_library(${PROJECT_NAME}_postprocessor
  src/datamatrix_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_postprocessor
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_postprocessor
  ${catkin_EXPORTED_TARGETS})


################# datamatrix handler #################
add_library(${PROJECT_NAME}_handler
  src/datamatrix_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_postprocessor
)

################# datamatrix node #################
add_executable(${PROJECT_NAME}_node
  src/datamatrix_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_handler
)

#~ To check lint : catkin_make roslint_pandora_vision_datamatrix
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_what_you_use")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/*
  src/*
  #test/unit/datamatrix_detector_test.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

################# Testing #################
if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()
