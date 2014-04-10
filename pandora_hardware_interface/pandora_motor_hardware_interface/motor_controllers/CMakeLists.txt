add_library(
  skid_steer_drive_controller
  src/skid_steer_drive_controller.cpp)
target_link_libraries(
  skid_steer_drive_controller
  ${catkin_LIBRARIES})
