cmake_minimum_required(VERSION 2.8.3)
project(pandora_geotiff)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  roslint
  std_msgs
  message_generation
  tf
  pandora_data_fusion_msgs
)

find_package(Qt4 4.6 COMPONENTS QtCore QtGui  REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${Eigen_DEFINITIONS})

include(${QT_USE_FILE})

add_service_files(
   FILES
   SaveMission.srv
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
    tf
    message_runtime
    pandora_data_fusion_msgs
  LIBRARIES
  DEPENDS Eigen QT
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(creator src/creator.cpp)
target_link_libraries(creator ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(server src/server.cpp)
target_link_libraries(server creator ${catkin_LIBRARIES})
add_dependencies(server
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )

add_executable(pandora_geotiff_node src/pandora_geotiff_node.cpp)
target_link_libraries(pandora_geotiff_node
      ${catkin_LIBRARIES}
      server
      creator
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

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
