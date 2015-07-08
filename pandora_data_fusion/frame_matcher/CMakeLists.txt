cmake_minimum_required(VERSION 2.8.3)
project(frame_matcher)

################################################################################
#                                 Find Package                                 #
################################################################################
# - Use `find_package` command to bring dependencies
# - Use `generate_dynamic_reconfigure_options` to setup dynamic reconfigure
# - Set flags if needed
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    nodelet
    pluginlib
    urdf
    tf
    pose_finder
    nav_msgs
    geometry_msgs
    sensor_processor
    pandora_vision_common
    pandora_data_fusion_utils
    cmake_modules
    pandora_cmake_tools
    roslint
  )

################################################################################
#                             Catkin Package Setup                             #
################################################################################
# - Use `catkin_package` to declare this catkin package and how it interacts
#   with other packages in the catkin package ecosystem
# - Use `catkin_python_setup` to setup catkin python ecosystem with ./setup.py
# - Set `include_directories`
# - Set `link_directories` (if necessary, usually avoid to if concerns catkin
#   libraries)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    urdf
    tf
    pose_finder
    nav_msgs
    geometry_msgs
    sensor_processor
    pandora_data_fusion_utils
    pandora_vision_common
  INCLUDE_DIRS
    include
  LIBRARIES
    enhanced_image_preprocessor
    enhanced_image_postprocessor
    view_pose_finder
    keypoint_transformer
    roi_transformer
    matcher_processor
    ${PROJECT_NAME}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

################################################################################
#                                  Libraries                                   #
################################################################################
# - Declare software libraries that will be made in this package and their
#   target names with `add_library`
# - Set `target_link_libraries` for each library target
# - Set `add_dependencies`, if necessary, to `${catkin_EXPORTED_TARGETS}` or
#   others

add_library(enhanced_image_preprocessor
  src/plugins/enhanced_image_preprocessor.cpp
  )
target_link_libraries(enhanced_image_preprocessor
  ${catkin_LIBRARIES}
  )
add_dependencies(enhanced_image_preprocessor
  ${catkin_EXPORTED_TARGETS}
  )

add_library(enhanced_image_postprocessor
  src/plugins/enhanced_image_postprocessor.cpp
  )
target_link_libraries(enhanced_image_postprocessor
  ${catkin_LIBRARIES}
  )
add_dependencies(enhanced_image_postprocessor
  ${catkin_EXPORTED_TARGETS}
  )

add_library(view_pose_finder
  src/view_pose_finder.cpp
  )
target_link_libraries(view_pose_finder
  ${catkin_LIBRARIES}
  )

add_library(keypoint_transformer
  src/keypoint_transformer.cpp
  )
target_link_libraries(keypoint_transformer
  ${catkin_LIBRARIES}
  view_pose_finder
  )

add_library(roi_transformer
  src/roi_transformer.cpp
  )
target_link_libraries(roi_transformer
  ${catkin_LIBRARIES}
  keypoint_transformer
  )

add_library(matcher_processor
  src/matcher_processor.cpp
  )
target_link_libraries(matcher_processor
  ${catkin_LIBRARIES}
  roi_transformer
  )

add_library(${PROJECT_NAME}
  src/frame_matcher.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  enhanced_image_preprocessor
  enhanced_image_postprocessor
  matcher_processor
  )

################################################################################
#                                 Executables                                  #
################################################################################
# - Declare software executables that will be made in this package and their
#   target names with `add_executable`
# - Set `target_link_libraries` for each executable target

add_executable(${PROJECT_NAME}_node
  src/frame_matcher_node.cpp
  )
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  )

################################################################################
#                                  Roslint
################################################################################
# - Setup roslint for test code with `roslint_cpp`, `roslint_python` or
# `roslint_custom`

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/**/*.h
    src/**/*.cpp
    src/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

################################################################################
#                                   Testing                                    #
################################################################################
# - Declare test subdirectory, in order not to pollute source code's
#   CMakeLists.txt with it

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

################################################################################
#                               Install Package                                #
################################################################################
# - Install directory of header files with `install(DIRECTORY ...)`
# - Install targets of exported libraries with `install(TARGETS ...)`
# - Install targets of executables with `install(TARGETS ...)`

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS
  enhanced_image_preprocessor
  enhanced_image_postprocessor
  view_pose_finder
  keypoint_transformer
  roi_transformer
  matcher_processor
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES frame_matcher_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
