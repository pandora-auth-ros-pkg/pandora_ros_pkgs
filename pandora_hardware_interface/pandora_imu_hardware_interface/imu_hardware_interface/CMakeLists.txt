add_library(
  imu_hardware_interface
  src/imu_hardware_interface.cpp)
target_link_libraries(
  imu_hardware_interface
  imu_drivers
  ${catkin_LIBRARIES})

add_executable(
  imu_hardware_interface_node
  src/imu_hardware_interface_node.cpp)
add_dependencies(
  imu_hardware_interface_node
  ${PROJECT_NAME}_gencfg)
target_link_libraries(
  imu_hardware_interface_node
  imu_hardware_interface
  ${catkin_LIBRARIES})


