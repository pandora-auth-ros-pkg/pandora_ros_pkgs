## --- Skeed Steer Drive controller --- ##
# TODO --- Fix gsl dependencies here!!!!
##########################################
add_library(
  skid_steer_drive_controller
  src/skid_steer_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_link_libraries(
  skid_steer_drive_controller
  ${catkin_LIBRARIES}
  ${GSL_LIBRARIES}
)
##########################################


## --- Skeed Steer Velocity controller --- ##
#############################################
add_library(
  skid_steer_velocity_controller
  src/skid_steer_velocity_controller.cpp
)

target_link_libraries(
  skid_steer_velocity_controller
  ${catkin_LIBRARIES}
  ${GSL_LIBRARIES}
)

add_dependencies(
  skid_steer_velocity_controller
  ${catkin_EXPORTED_TARGETS}
)
#############################################


## --- Skeed Steer Toruqe controller --- ##
###########################################
add_library(
  skid_steer_torque_controller
  src/skid_steer_torque_controller.cpp
)

target_link_libraries(
  skid_steer_torque_controller
  ${catkin_LIBRARIES}
)

add_dependencies(
  skid_steer_torque_controller
  ${catkin_EXPORTED_TARGETS}
)
##########################################
