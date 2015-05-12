cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_qrcode)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_vision_common
    pandora_vision_msgs
    pandora_common_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    sensor_msgs
    roslint
    urdf
    pandora_cmake_tools
)

find_package(PkgConfig)
pkg_check_modules(ZBAR zbar)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_vision_common
      pandora_vision_msgs
      pandora_common_msgs
      state_manager
      state_manager_msgs
      cv_bridge
      image_transport
      sensor_msgs
      urdf
    #INCLUDE_DIRS
    #LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################# qrcode detector #################
add_library(${PROJECT_NAME}_detector
  src/qrCode_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_detector
  ${catkin_LIBRARIES}
  zbar
)

################# qrcode preprocessor #################
add_library(${PROJECT_NAME}_preprocessor
  src/qrcode_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_preprocessor
  ${catkin_LIBRARIES}
)

################# qrcode postprocessor #################
add_library(${PROJECT_NAME}_postprocessor
  src/qrcode_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_postprocessor
  ${catkin_LIBRARIES}
)

################# qrcode handler #################
add_library(${PROJECT_NAME}_handler
  src/qrcode_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_postprocessor
)

################# qrcode node #################
add_executable(${PROJECT_NAME}_node
  src/qrcode_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_handler
)


################# roslint #################

#~ To check lint : catkin_make roslint_pandora_vision_qrcode
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line,-build/include_what_you_use, -runtime/references")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/*.h
  src/*.cpp
  test/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

################################################################################
#                                   Testing                                    #
################################################################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
