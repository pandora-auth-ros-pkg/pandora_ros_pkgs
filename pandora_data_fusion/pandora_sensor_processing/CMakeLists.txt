cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_processing)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  roscpp
  state_manager
  state_manager_msgs
  dynamic_reconfigure
  roslint
  pandora_arm_hardware_interface
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
    cmake_modules
    roscpp
    state_manager
    state_manager_msgs
    dynamic_reconfigure
    roslint
    pandora_arm_hardware_interface
    pandora_common_msgs
    sensor_msgs
  INCLUDE_DIRS
    include
    ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
  )

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  )

########################  utils  ####################################

add_library(sensor_utils src/utils.cpp)
target_link_libraries(sensor_utils
  ${catkin_LIBRARIES}
  )

########################  co2_processor  ############################

add_library(co2_processor src/co2_processor.cpp)
target_link_libraries(co2_processor
  ${catkin_LIBRARIES}
  sensor_utils
  )
add_dependencies(co2_processor
  pandora_sensor_processing_gencfg
  pandora_arm_hardware_interface_gencpp
  pandora_state_manager_msgs_gencpp
  pandora_common_msgs_gencpp
  )

########################  co2_node  #################################

add_executable(co2_node src/co2_node.cpp)
target_link_libraries(co2_node
  ${catkin_LIBRARIES}
  co2_processor
  )

########################  clusterer  ################################

add_library(clusterer src/clusterer.cpp)
target_link_libraries(clusterer
  ${catkin_LIBRARIES}
  sensor_utils
  )

########################  thermal_processor  ########################

add_library(thermal_processor src/thermal_processor.cpp)
target_link_libraries(thermal_processor
  ${catkin_LIBRARIES}
  sensor_utils
  clusterer
  )
add_dependencies(thermal_processor
  pandora_sensor_processing_gencfg
  pandora_arm_hardware_interface_gencpp
  pandora_state_manager_msgs_gencpp
  pandora_common_msgs_gencpp
  )

########################  thermal_node  #############################

add_executable(thermal_node src/thermal_node.cpp)
target_link_libraries(thermal_node
  ${catkin_LIBRARIES}
  thermal_processor
  )

########################  roslint  ##################################

set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
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

