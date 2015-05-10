add_library(
  leddar_serial_interface 
  src/leddar_serial_interface.cpp
  src/OS.cpp
  src/Modbus.cpp
  src/Leddar.cpp
  )

target_link_libraries(
  leddar_serial_interface
  ${catkin_LIBRARIES}
  )
  
add_executable(
  leddar_serial_interface_demo
  src/leddar_serial_interface_demo.cpp)

target_link_libraries(
  leddar_serial_interface_demo
  leddar_serial_interface
  ${catkin_LIBRARIES}
  )
