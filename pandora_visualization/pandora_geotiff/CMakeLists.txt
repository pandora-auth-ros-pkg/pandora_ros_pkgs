cmake_minimum_required(VERSION 2.8.3)
project(pandora_geotiff)
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  pluginlib
  roscpp
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
    pluginlib
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

add_library(geotiff_creator src/geotiff_creator.cpp)
target_link_libraries(geotiff_creator ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(qr_csv_creator src/qr_csv_creator.cpp)
target_link_libraries(qr_csv_creator ${catkin_LIBRARIES})
add_dependencies(qr_csv_creator ${catkin_EXPORTED_TARGETS})

add_library(map_generator src/map_generator.cpp)
target_link_libraries(map_generator  geotiff_creator ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(map_generator ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(pandora_geotiff_node src/pandora_geotiff_node.cpp)
target_link_libraries(pandora_geotiff_node
      ${catkin_LIBRARIES}
      map_generator
      geotiff_creator
      qr_csv_creator
      )

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
