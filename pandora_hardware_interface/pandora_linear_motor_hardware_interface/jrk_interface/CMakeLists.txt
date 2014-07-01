add_library(jrk_serial src/JrkSerial.cpp)

target_link_libraries(jrk_serial ${catkin_LIBRARIES})

#add_executable(pololu_jrk_communicator_node src/JrkCommunicator.cpp src/JrkSerial.cpp)

#target_link_libraries(pololu_jrk_communicator_node jrk_serial ${catkin_LIBRARIES})
