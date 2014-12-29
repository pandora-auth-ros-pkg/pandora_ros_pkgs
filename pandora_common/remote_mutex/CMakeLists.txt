cmake_minimum_required(VERSION 2.8.3)
project(remote_mutex)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  remote_mutex_msgs)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      remote_mutex_msgs
    INCLUDE_DIRS include
    LIBRARIES
      remote_mutex
      mutex_guard
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mutex src/standalone_mutex.cpp src/mutex_guard.cpp)
target_link_libraries(mutex ${catkin_LIBRARIES} )
add_dependencies(mutex ${catkin_EXPORTED_TARGETS})

add_library(remote_mutex src/remote_mutex.cpp)
target_link_libraries(remote_mutex ${catkin_LIBRARIES} )
add_dependencies(remote_mutex ${catkin_EXPORTED_TARGETS})

add_library(mutex_guard src/mutex_guard.cpp)
target_link_libraries(mutex_guard ${catkin_LIBRARIES} )
add_dependencies(mutex_guard ${catkin_EXPORTED_TARGETS})

