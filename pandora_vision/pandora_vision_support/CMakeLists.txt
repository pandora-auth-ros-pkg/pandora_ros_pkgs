cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_support)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  pandora_vision_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  pandora_vision_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(image_saver_by_topic_node
  src/image_saver_by_topic.cpp
  )
target_link_libraries(image_saver_by_topic_node
  ${catkin_LIBRARIES}
  )

add_executable(enhanced_image_shower_node
  src/enhanced_image_shower.cpp
  )
target_link_libraries(enhanced_image_shower_node
  ${catkin_LIBRARIES}
  )
add_dependencies(enhanced_image_shower_node
  ${catkin_EXPORTED_TARGETS}
  )
