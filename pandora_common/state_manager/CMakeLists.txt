cmake_minimum_required(VERSION 2.8.3)
project(state_manager)

find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    std_msgs
                    actionlib
                    state_manager_communications
                    )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
   LIBRARIES state_client 
   DEPENDS 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
)

add_executable(state_changer src/state_changer.cpp src/state_client.cpp)
target_link_libraries(state_changer ${catkin_LIBRARIES} )
add_dependencies(state_changer state_manager_communications_gencpp)

add_executable(state_server src/state_server.cpp)
target_link_libraries(state_server ${catkin_LIBRARIES}  )
add_dependencies(state_server state_manager_communications_gencpp)

add_library(state_client src/state_client.cpp)
target_link_libraries(state_client ${catkin_LIBRARIES} )
add_dependencies(state_client state_manager_communications_gencpp)

