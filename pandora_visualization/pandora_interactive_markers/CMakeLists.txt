cmake_minimum_required(VERSION 2.8.3)
project(pandora_interactive_markers)

find_package(catkin REQUIRED COMPONENTS
      roscpp
      visualization_msgs
      tf
      sensor_msgs
      interactive_markers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      visualization_msgs
      tf
      sensor_msgs
      interactive_markers
    #INCLUDE_DIRS
    #LIBRARIES
)


add_executable(
  interactive_tf_publisher_marker
  src/interactive_tf_publisher_marker.cpp)
target_link_libraries(
  interactive_tf_publisher_marker
  ${catkin_LIBRARIES} )

add_executable(
  interactive_command_publisher_marker
  src/interactive_command_publisher_marker.cpp)
target_link_libraries(
  interactive_command_publisher_marker
  ${catkin_LIBRARIES} )
