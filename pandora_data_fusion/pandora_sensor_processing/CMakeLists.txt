cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_processing)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  state_manager
  state_manager_msgs
  dynamic_reconfigure
  roslint
  pandora_sensor_msgs
  pandora_common_msgs
  sensor_msgs
  )

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  config/sensor_processing/SensorProcessing.cfg
  )

catkin_package(
  DEPENDS
    Eigen
  CATKIN_DEPENDS
    roscpp
    state_manager
    state_manager_msgs
    dynamic_reconfigure
    pandora_sensor_msgs
    pandora_common_msgs
    sensor_msgs
  #INCLUDE_DIRS
  #LIBRARIES
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  )

########################  utils  ####################################

add_library(${PROJECT_NAME}_sensor_utils src/utils.cpp)
target_link_libraries(${PROJECT_NAME}_sensor_utils
  ${catkin_LIBRARIES}
  )

########################  co2_processor  ############################

add_library(${PROJECT_NAME}_co2_processor src/co2_processor.cpp)
target_link_libraries(${PROJECT_NAME}_co2_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sensor_utils
  )
add_dependencies(${PROJECT_NAME}_co2_processor
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

########################  co2_node  #################################

add_executable(co2_node src/co2_node.cpp)
target_link_libraries(co2_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_co2_processor
  )

########################  clusterer  ################################

add_library(${PROJECT_NAME}_clusterer src/clusterer.cpp)
target_link_libraries(${PROJECT_NAME}_clusterer
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sensor_utils
  )

########################  thermal_processor  ########################

add_library(${PROJECT_NAME}_thermal_processor src/thermal_processor.cpp)
target_link_libraries(${PROJECT_NAME}_thermal_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sensor_utils
  ${PROJECT_NAME}_clusterer
  )
add_dependencies(${PROJECT_NAME}_thermal_processor
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

########################  thermal_node  #############################

add_executable(thermal_node src/thermal_node.cpp)
target_link_libraries(thermal_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_thermal_processor
  )

########################  roslint  ##################################

set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR}
            include/sensor_processing/*.h
            src/*.cpp
            test/unit/*.cpp
            test/functional/*.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

########################  testing  ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()


##################### Install targets ###############################

## Install exported libraries
install(TARGETS ${PROJECT_NAME}_thermal_processor
                ${PROJECT_NAME}_co2_processor
                ${PROJECT_NAME}_sensor_utils
                ${PROJECT_NAME}_clusterer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Install executables
install(TARGETS thermal_node co2_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
