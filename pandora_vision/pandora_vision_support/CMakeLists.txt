cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_support)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(image_saver_by_topic_node
  src/image_saver_by_topic.cpp
)

target_link_libraries(image_saver_by_topic_node
  ${catkin_LIBRARIES}
)

