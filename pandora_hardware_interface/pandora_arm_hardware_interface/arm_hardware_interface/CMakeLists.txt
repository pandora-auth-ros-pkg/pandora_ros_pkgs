add_executable(
  arm_hardware_interface_node
  src/arm_hardware_interface.cpp
  src/arm_hardware_interface_node.cpp)

target_link_libraries(
  arm_hardware_interface_node
  arm_usb_interface
  ${catkin_LIBRARIES})
