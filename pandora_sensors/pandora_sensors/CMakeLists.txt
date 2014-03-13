cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensors)
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    actionlib_msgs 
    actionlib 
    controllers_and_sensors_communications
    diagnostic_updater 
    tf
    serial
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
      roscpp
      actionlib_msgs 
      actionlib 
      controllers_and_sensors_communications
      diagnostic_updater 
      tf
      serial
    INCLUDE_DIRS include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uController 
  src/uController/uController_main.cpp 
  src/uController/uController.cpp 
  src/uController/sensorHandler.cpp 
  src/uController/hardware_interface.cpp 
  src/uController/compTiny.cpp 
  src/uController/tpa81.cpp 
  src/uController/srf05Tiny.cpp
  src/uController/battery.cpp 
  src/uController/aggregateSensors.cpp)
target_link_libraries(uController ${catkin_LIBRARIES} )
add_dependencies(uController 
  controllers_and_sensors_communications_gencpp
)

add_executable(head_node src/head_node/head_node.cpp)
target_link_libraries(head_node ${catkin_LIBRARIES} )
add_dependencies(head_node 
    controllers_and_sensors_communications_gencpp
)

