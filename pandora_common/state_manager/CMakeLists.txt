cmake_minimum_required(VERSION 2.8.3)
project(state_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  diagnostic_updater
  state_manager_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    diagnostic_updater
    state_manager_msgs
  LIBRARIES
    state_client
  #DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_changer src/state_changer.cpp src/state_client.cpp)
target_link_libraries(state_changer ${catkin_LIBRARIES} )
add_dependencies(state_changer ${catkin_EXPORTED_TARGETS})

add_executable(state_server src/state_server.cpp)
target_link_libraries(state_server ${catkin_LIBRARIES}  )
add_dependencies(state_server ${catkin_EXPORTED_TARGETS})

add_library(state_client src/state_client.cpp)
target_link_libraries(state_client ${catkin_LIBRARIES} )
add_dependencies(state_client ${catkin_EXPORTED_TARGETS})

