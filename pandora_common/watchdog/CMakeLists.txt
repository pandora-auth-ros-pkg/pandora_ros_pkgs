cmake_minimum_required(VERSION 2.8.3)
project(watchdog)
find_package(catkin REQUIRED COMPONENTS roscpp watchdog_msgs)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp watchdog_msgs
    INCLUDE_DIRS include
    LIBRARIES watchdog
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(watchdog_monitor src/watchdog_monitor.cpp)
target_link_libraries(watchdog_monitor ${catkin_LIBRARIES} )
add_dependencies(watchdog_monitor ${catkin_EXPORTED_TARGETS})

add_library(watchdog src/watchdog.cpp)
target_link_libraries(watchdog ${catkin_LIBRARIES} )
add_dependencies(watchdog ${catkin_EXPORTED_TARGETS})
