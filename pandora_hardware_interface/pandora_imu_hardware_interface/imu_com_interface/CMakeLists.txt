add_library(
  imu_drivers
  src/imu_com_interface.cpp
  src/ahrs_com_interface.cpp)
target_link_libraries(
  imu_drivers
  ${catkin_LIBRARIES})
  
add_executable(
  trax_ahrs_configuration_node
  src/trax_ahrs_configuration_node.cpp)
target_link_libraries(
  trax_ahrs_configuration_node
  imu_drivers
  ${catkin_LIBRARIES})

