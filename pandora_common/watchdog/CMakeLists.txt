cmake_minimum_required(VERSION 2.8.3)
project(watchdog)
find_package(catkin REQUIRED COMPONENTS roscpp watchdog_communications)

catkin_package(
    DEPENDS  
    CATKIN-DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES watchdog
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_executable(watchdog_monitor src/watchdog_monitor.cpp)
target_link_libraries(watchdog_monitor ${catkin_LIBRARIES} )
add_dependencies(watchdog_monitor watchdog_communications_gencpp)

add_library(watchdog src/watchdog.cpp)
target_link_libraries(watchdog ${catkin_LIBRARIES} )
add_dependencies(watchdog watchdog_communications_gencpp)



