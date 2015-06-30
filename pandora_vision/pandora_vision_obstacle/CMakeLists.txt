cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_obstacle)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    dynamic_reconfigure
    cv_bridge
    sensor_msgs
    pandora_vision_msgs
    pandora_vision_common
    sensor_processor
    state_manager
    state_manager_msgs
    pandora_cmake_tools
)

generate_dynamic_reconfigure_options(
  config/pandora_vision_obstacle/soft_obstacle_cfg.cfg
)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      dynamic_reconfigure
      cv_bridge
      sensor_msgs
      pandora_vision_msgs
      pandora_vision_common
      sensor_processor
      state_manager
      state_manager_msgs
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

############################# discrete wavelet transform #########################
add_library(${PROJECT_NAME}_discrete_wavelet_transform
  src/discrete_wavelet_transform.cpp
)
target_link_libraries(${PROJECT_NAME}_discrete_wavelet_transform
  ${catkin_LIBRARIES}
)

############################### obstacle preprocessor ##############################
add_library(${PROJECT_NAME}_obstacle_preprocessor
  src/obstacle_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_obstacle_preprocessor
  ${catkin_LIBRARIES}
)

############################### obstacle postprocessor ##############################
add_library(${PROJECT_NAME}_obstacle_postprocessor
  src/obstacle_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_obstacle_postprocessor
  ${catkin_LIBRARIES}
)

############################### barrel processor ##############################
add_library(${PROJECT_NAME}_barrel_processor
  src/barrel_detection/barrel_processor.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_processor
  ${catkin_LIBRARIES}
)

############################### barrel handler ##############################
add_library(${PROJECT_NAME}_barrel_handler
  src/barrel_detection/barrel_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_barrel_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_obstacle_preprocessor
  ${PROJECT_NAME}_obstacle_postprocessor
  ${PROJECT_NAME}_barrel_processor
)

############################### soft obstacle detector ##############################
add_library(${PROJECT_NAME}_soft_obstacle_detector
  src/soft_obstacle_detection/soft_obstacle_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_soft_obstacle_detector
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_discrete_wavelet_transform
)

############################### soft obstacle processor ##############################
add_library(${PROJECT_NAME}_soft_obstacle_processor
  src/soft_obstacle_detection/soft_obstacle_processor.cpp
)
target_link_libraries(${PROJECT_NAME}_soft_obstacle_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_soft_obstacle_detector
)

############################### soft obstacle handler ##############################
add_library(${PROJECT_NAME}_soft_obstacle_handler
  src/soft_obstacle_detection/soft_obstacle_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_soft_obstacle_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_obstacle_preprocessor
  ${PROJECT_NAME}_obstacle_postprocessor
  ${PROJECT_NAME}_soft_obstacle_processor
)

############################ barrel node ###############################
add_executable(${PROJECT_NAME}_barrel_node
  src/barrel_detection/barrel_node.cpp
)
add_dependencies(${PROJECT_NAME}_barrel_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_barrel_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_barrel_handler
)

############################ soft obstacle node ###############################
add_executable(${PROJECT_NAME}_soft_obstacle_node
  src/soft_obstacle_detection/soft_obstacle_node.cpp
)
add_dependencies(${PROJECT_NAME}_soft_obstacle_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_soft_obstacle_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_soft_obstacle_handler
)

#~ To check lint : catkin_make roslint_pandora_vision_obstacle

#set(ROSLINT_CPP_OPTS
    #"--filter=-build/include_what_you_use")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  src/*.cpp
  test/*.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################################### Testing ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
