cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_landoltc)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_common_msgs
    pandora_vision_msgs
    pandora_vision_common
    pandora_cmake_tools
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
  config/pandora_vision_landoltc/landoltc_cfg.cfg
  config/pandora_vision_landoltc/landoltc3d_cfg.cfg
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
      sensor_msgs
      dynamic_reconfigure
      urdf
    #INCLUDE_DIRS
    #LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

########################### landoltc detector ############################

add_library(${PROJECT_NAME}_2d_detector
  src/landoltc_2d/landoltc_detector.cpp
  src/landoltc_2d/landoltc_parameters.cpp
  )
target_link_libraries(${PROJECT_NAME}_2d_detector 
  ${catkin_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}_2d_detector
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

########################### landoltc preprocessor ############################

add_library(${PROJECT_NAME}_2d_preprocessor
  src/landoltc_2d/landoltc_preprocessor.cpp
  )
target_link_libraries(${PROJECT_NAME}_2d_preprocessor 
  ${catkin_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME}_2d_preprocessor
  ${catkin_EXPORTED_TARGETS})
########################### landoltc postprocessor ############################

add_library(${PROJECT_NAME}_2d_postprocessor
  src/landoltc_2d/landoltc_postprocessor.cpp
  )
target_link_libraries(${PROJECT_NAME}_2d_postprocessor 
  ${catkin_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME}_2d_postprocessor
  ${catkin_EXPORTED_TARGETS})
########################### landoltc handler ############################

add_library(${PROJECT_NAME}_2d_handler
  src/landoltc_2d/landoltc_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_2d_handler 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_2d_detector
  ${PROJECT_NAME}_2d_preprocessor
  ${PROJECT_NAME}_2d_postprocessor
  )

########################### landoltcNode ############################
add_executable(${PROJECT_NAME}_2d_node
  src/landoltc_2d/landoltc_node.cpp
)
target_link_libraries(${PROJECT_NAME}_2d_node 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_2d_handler
  )
add_dependencies(${PROJECT_NAME}_2d_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


add_library(${PROJECT_NAME}_3d_lib
  src/landoltc_3d/landoltc3d_detector.cpp
  src/landoltc_3d/landoltc3d_parameters.cpp)

target_link_libraries(${PROJECT_NAME}_3d_lib ${catkin_LIBRARIES})

########################### landoltc3dNode ############################
add_executable(${PROJECT_NAME}_3d_node
              src/landoltc_3d/landoltc3d_parameters.cpp
              src/landoltc_3d/landoltc3d_node.cpp
              src/landoltc_3d/landoltc3d_detection.cpp
              src/landoltc_3d/landoltc3d_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_3d_node ${catkin_LIBRARIES} )

add_dependencies(${PROJECT_NAME}_3d_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


#~ To check lint : catkin_make roslint_pandora_vision_landoltc
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_what_you_use, -whitespace/empty_loop_body")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  
  include/landoltc_2d/*
  src/landoltc_2d/*
  #~ include/landoltc_3d/landoltc3d_detector.h
  #~ include/landoltc_3d/landoltc3d_detection.h
  #~ include/landoltc_2d/landoltc_detector.h
  #~ include/landoltc_2d/landoltc_detection.h
  #~ include/landoltc_2d/landoltc_parameters.h
  #~ include/landoltc_3d/landoltc3d_parameters.h
  #~ src/landoltc_2d/landoltc_detector.cpp
  #~ src/landoltc_3d/landoltc3d_detector.cpp
  #~ src/landoltc_3d/landoltc3d_detection.cpp
  #~ src/landoltc_3d/landoltc3d_node.cpp
  #~ src/landoltc_2d/landoltc_detection.cpp
  #~ src/landoltc_2d/landoltc_node.cpp
  #~ src/landoltc_2d/landoltc_parameters.cpp
  #~ src/landoltc_3d/landoltc3d_parameters.cpp
)

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_TEST
  RELATIVE ${PROJECT_SOURCE_DIR}
    test/unit/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_TEST})

##################################### Testing ##################################

if(CATKIN_ENABLE_TESTING)

  if (${PROJECT_NAME}_download)
      set(${PROJECT_NAME}_download TRUE)
  endif()

  add_subdirectory(test)
endif()
