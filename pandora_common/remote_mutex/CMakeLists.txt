cmake_minimum_required(VERSION 2.8.3)
project(remote_mutex)
find_package(catkin REQUIRED COMPONENTS
                      roscpp
                      remote_mutex_communications)
catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES remote_mutex
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

add_executable(mutex src/standalone_mutex.cpp src/mutex_guard.cpp)
target_link_libraries(mutex ${catkin_LIBRARIES} )
add_dependencies(mutex remote_mutex_communications_gencpp)

add_library(remote_mutex src/remote_mutex.cpp)
target_link_libraries(remote_mutex ${catkin_LIBRARIES} )
add_dependencies(remote_mutex remote_mutex_communications_gencpp)

add_library(mutex_guard src/mutex_guard.cpp)
target_link_libraries(mutex_guard ${catkin_LIBRARIES} )
add_dependencies(mutex_guard remote_mutex_communications_gencpp)

