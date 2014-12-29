cmake_minimum_required(VERSION 2.8.3)
project(remote_mutex_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(
  FILES
    mutexSrv.srv
)

generate_messages()

catkin_package(
  #DEPENDS
  CATKIN_DEPENDS message_runtime
  #INCLUDE_DIRS
  #LIBRARIES
)
