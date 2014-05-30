add_library(epos_gateway
  src/epos_gateway/epos_serial_gateway.cpp 
)
target_link_libraries(epos_gateway ${catkin_LIBRARIES} )

add_library(serial_epos_handler
  src/epos_handler/abstract_epos_handler.cpp
  src/epos_handler/serial_epos_handler.cpp
)
target_link_libraries(serial_epos_handler
  epos_gateway
  ${catkin_LIBRARIES}
)

add_executable(epos_test
  src/epos_handler/test.cpp
)
target_link_libraries(epos_test
  serial_epos_handler
  ${catkin_LIBRARIES}
)



