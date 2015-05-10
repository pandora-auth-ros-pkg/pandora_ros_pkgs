link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(
  motor_hardware_interface_node
  src/motor_hardware_interface.cpp
  src/motor_hardware_interface_node.cpp)
target_link_libraries(
  motor_hardware_interface_node
  serial_epos_handler
  serial_epos2_handler
  ${catkin_LIBRARIES})
