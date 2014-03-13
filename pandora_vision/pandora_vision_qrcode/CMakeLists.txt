cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_qrcode)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules(ZBAR zbar)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      vision_communications
      state_manager
      state_manager_communications
      cv_bridge
      image_transport
      sensor_msgs
    INCLUDE_DIRS
      include
    LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qrcode_node
    src/qrCode_detection.cpp
    src/qrCode_detector.cpp
    src/qrCode_node.cpp
)
add_dependencies(qrcode_node 
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(qrcode_node ${catkin_LIBRARIES} zbar)
