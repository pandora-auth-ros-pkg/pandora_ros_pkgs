add_library(orangutan_serial src/orangutan_serial_interface.cpp)
target_link_libraries(orangutan_serial ${catkin_LIBRARIES}) 

add_executable(motor_test src/test.cpp)
target_link_libraries(motor_test orangutan_serial ${catkin_LIBRARIES})
