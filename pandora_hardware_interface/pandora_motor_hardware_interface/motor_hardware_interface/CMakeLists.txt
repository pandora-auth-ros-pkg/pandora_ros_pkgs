add_executable(
  motor_hardware_interface_node
  src/motor_hardware_interface.cpp
  src/motor_hardware_interface_node.cpp)
target_link_libraries(
  motor_hardware_interface_node
  orangutan_serial
  ${catkin_LIBRARIES})
