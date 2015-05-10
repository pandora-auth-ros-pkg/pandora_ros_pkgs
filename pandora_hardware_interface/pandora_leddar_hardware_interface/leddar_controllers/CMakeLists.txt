add_library(
  leddar_sensor_controller
  src/leddar_sensor_controller.cpp)

add_dependencies(
  leddar_sensor_controller
  ${PROJECT_NAME}_gencpp)

target_link_libraries(
  leddar_sensor_controller
  ${catkin_LIBRARIES})
