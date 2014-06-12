add_executable(
  linear_movement_planner
  src/linear_movement_planner.cpp)
add_dependencies(
  linear_movement_planner
  ${PROJECT_NAME}_gencpp)
target_link_libraries(
  linear_movement_planner 
  ${catkin_LIBRARIES} )
