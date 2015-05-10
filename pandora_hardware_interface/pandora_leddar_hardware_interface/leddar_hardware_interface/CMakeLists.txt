add_executable(
  leddar_hardware_interface_node
  src/leddar_hardware_interface.cpp
  src/leddar_hardware_interface_node.cpp
  )
  
target_link_libraries(
  leddar_hardware_interface_node
  leddar_serial_interface
  ${catkin_LIBRARIES}
  )
