add_library(
  arm_usb_interface
  src/arm_usb_interface.cpp)
target_link_libraries(
  arm_usb_interface
  ${catkin_LIBRARIES})

add_executable(
  arm_demo
  src/arm_usb_interface_demo.cpp)
target_link_libraries(
  arm_demo
  arm_usb_interface
  ${catkin_LIBRARIES})
