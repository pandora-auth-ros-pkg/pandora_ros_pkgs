add_executable(
  xmega_hardware_interface_node
  src/xmega_hardware_interface.cpp
  src/xmega_hardware_interface_node.cpp)
target_link_libraries(
  xmega_hardware_interface_node
  xmega_interface
  ${catkin_LIBRARIES})
