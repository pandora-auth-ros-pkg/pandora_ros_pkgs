add_library(
  battery_controller
  src/battery_controller.cpp)
add_dependencies(
  battery_controller
  ${PROJECT_NAME}_gencpp)
target_link_libraries(
  battery_controller
  ${catkin_LIBRARIES})

add_library(
  range_sensor_controller
  src/range_sensor_controller.cpp)
target_link_libraries(
  range_sensor_controller
  ${catkin_LIBRARIES})
