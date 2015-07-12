cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_coverage)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pandora_data_fusion_utils
  state_manager
  state_manager_msgs
  octomap_msgs
  octomap_ros
  roslint
  )

find_package(octomap REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    pandora_data_fusion_utils
    state_manager
    state_manager_msgs
    octomap_msgs
    octomap_ros
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}
    ${PROJECT_NAME}_sensor
    ${PROJECT_NAME}_surface_checker
    ${PROJECT_NAME}_coverage_checker
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )
link_directories(
  ${OCTOMAP_LIBRARY_DIRS}
  )

########################  utils  ####################################

add_library(${PROJECT_NAME}_utils
  src/utils.cpp
  )
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )

########################  coverage checker  #########################

add_library(${PROJECT_NAME}_coverage_checker
  src/coverage_checker.cpp)
target_link_libraries(${PROJECT_NAME}_coverage_checker
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  )

########################  space checker  ############################

#add_library(${PROJECT_NAME}_space_checker
  #src/space_checker.cpp)
#target_link_libraries(${PROJECT_NAME}_space_checker
  #${catkin_LIBRARIES}
  #${PROJECT_NAME}_coverage_checker
  #)

########################  surface checker  ##########################

add_library(${PROJECT_NAME}_surface_checker
  src/surface_checker.cpp)
target_link_libraries(${PROJECT_NAME}_surface_checker
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_coverage_checker
  )

########################  sensor  ###################################

add_library(${PROJECT_NAME}_sensor
  src/sensor.cpp)
add_dependencies(${PROJECT_NAME}_sensor
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_sensor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_surface_checker
  )

########################  sensor_coverage  ##########################

add_library(${PROJECT_NAME}
  src/sensor_coverage.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sensor
  ${PROJECT_NAME}_utils
  )

########################  sensor_coverage_node  #####################

add_executable(${PROJECT_NAME}_node
  src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )

########################  roslint  ##################################

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  include/sensor_coverage/*.h
  src/*.cpp
  test/unit/*.cpp
  test/functional/*.cpp
  )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

########################  testing  ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

################################  Roslint  #####################################

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/**/*.h
    include/**/*.hxx
    src/**/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################### Install targets ###############################

## Install exported libraries
install(TARGETS ${PROJECT_NAME}_utils
                ${PROJECT_NAME}
                ${PROJECT_NAME}_sensor
                ${PROJECT_NAME}_surface_checker
                ${PROJECT_NAME}_coverage_checker
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Install executables
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
