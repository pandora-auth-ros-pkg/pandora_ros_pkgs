add_executable(
  sensor_orientation_planner
  src/sensor_orientation_planner.cpp)
add_dependencies(
  sensor_orientation_planner
  ${PROJECT_NAME}_gencpp)
target_link_libraries(
  sensor_orientation_planner 
  ${catkin_LIBRARIES} )
