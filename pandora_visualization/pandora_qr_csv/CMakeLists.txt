cmake_minimum_required(VERSION 2.8.3)
project(pandora_qr_csv)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslint
  std_msgs
  message_generation
  pandora_data_fusion_msgs
  pandora_geotiff
)

find_package(Eigen REQUIRED)
add_definitions(${Eigen_DEFINITIONS})

add_service_files(
   FILES
   createCSV.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    pandora_data_fusion_msgs
    pandora_geotiff
  LIBRARIES
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(qr_csv_creator src/qr_csv_creator.cpp)
target_link_libraries(qr_csv_creator ${catkin_LIBRARIES})
add_dependencies(qr_csv_creator ${catkin_EXPORTED_TARGETS})

add_executable(pandora_qr_csv_node src/pandora_qr_csv_node.cpp)
target_link_libraries(pandora_qr_csv_node
      ${catkin_LIBRARIES}
      qr_csv_creator
      )

############################## ROSLINT ##############################

set(ROSLINT_CPP_OPTS
    "--filter=-runtime/references, -runtime/threadsafe_fn")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
            ${PROJECT_SOURCE_DIR}
            include/**/*.h
            src/*.cpp
            test/**/*.cpp
        )

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

############################## TESTING ##############################

#if(CATKIN_ENABLE_TESTING)
  #add_subdirectory(test)
#endif()
