add_library(
  xmega_interface
  src/battery_sensor.cpp
  src/encoder_sensor.cpp
  src/range_sensor.cpp
  src/xmega_serial_interface.cpp)
target_link_libraries(
  xmega_interface
  ${catkin_LIBRARIES})

add_executable(
  xmega_test
  src/test.cpp)
target_link_libraries(
  xmega_test
  xmega_interface
  ${catkin_LIBRARIES})
