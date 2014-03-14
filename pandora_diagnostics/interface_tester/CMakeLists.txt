cmake_minimum_required(VERSION 2.8.3)
project(interface_tester)
find_package(catkin REQUIRED COMPONENTS 
            roscpp 
            tf
            )
include_directories(include ${catkin_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS roscpp tf
    INCLUDE_DIRS include
    LIBRARIES interface_tester tf_monitor 
)

add_library(interface_tester src/interface_tester.cpp)
target_link_libraries(interface_tester ${catkin_LIBRARIES} )

add_library(tf_monitor src/tf_monitor.cpp)
target_link_libraries(tf_monitor ${catkin_LIBRARIES})



