cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_qrcode)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_vision_common
    pandora_vision_msgs
    pandora_common_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    sensor_msgs
    roslint
    urdf
    pandora_cmake_tools
)

find_package(PkgConfig)
pkg_check_modules(ZBAR zbar)

generate_dynamic_reconfigure_options(
  config/pandora_vision_qrcode/qrcode_cfg.cfg
)

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
      dynamic_reconfigure
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
  src/qrcode_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_detector
  ${catkin_LIBRARIES}
  zbar
)
add_dependencies(${PROJECT_NAME}_detector
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

################# qrcode processor ###################
add_library(${PROJECT_NAME}_processor
  src/qrcode_processor.cpp
)
target_link_libraries(${PROJECT_NAME}_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_detector
)
add_dependencies(${PROJECT_NAME}_processor
  ${PROJECT_NAME}_detector
  ${catkin_EXPORTED_TARGETS})

################# qrcode preprocessor #################
add_library(${PROJECT_NAME}_preprocessor
  src/qrcode_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_preprocessor
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_preprocessor
  ${catkin_EXPORTED_TARGETS})

################# qrcode postprocessor #################
add_library(${PROJECT_NAME}_postprocessor
  src/qrcode_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_postprocessor
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_postprocessor
  ${catkin_EXPORTED_TARGETS})
################# qrcode handler #################
add_library(${PROJECT_NAME}_handler
  src/qrcode_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_processor
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
    "--filter=-runtime/references")
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
  # Check if the benchmark images should be downloaded.
  if(DOWNLOAD_TESTING_DATA)
    set(${PROJECT_NAME}_download_testing_data true)
  endif()
  if(BENCHMARK)
    set(${PROJECT_NAME}_benchmark true)
  endif()
  add_subdirectory(test)
endif()
