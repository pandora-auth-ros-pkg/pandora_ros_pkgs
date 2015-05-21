cmake_minimum_required(VERSION 2.8.3)
project(pandora_geotiff_plugins)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  nav_msgs
  tf
  pandora_geotiff
  pandora_data_fusion_msgs
)


catkin_package(
#INCLUDE_DIRS
LIBRARIES
  pandora_geotiff_plugins
CATKIN_DEPENDS
  pluginlib
  nav_msgs
  tf
  pandora_geotiff
  pandora_data_fusion_msgs
#DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(pandora_geotiff_plugins
  src/geotiff_map_plugin.cpp
  src/geotiff_coverage_plugin.cpp
  src/geotiff_path_plugin.cpp
  src/geotiff_objects_plugin.cpp
)
add_dependencies(pandora_geotiff_plugins   ${catkin_EXPORTED_TARGETS})
target_link_libraries(pandora_geotiff_plugins 
  ${catkin_LIBRARIES}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  pandora_geotiff_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

