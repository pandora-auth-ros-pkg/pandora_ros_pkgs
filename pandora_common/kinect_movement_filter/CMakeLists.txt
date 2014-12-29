cmake_minimum_required(VERSION 2.8.3)
project(kinect_movement_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  roslint
)

catkin_package(
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    image_transport
#  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
  kinect_movement_filter
  src/kinect_movement_filter.cpp)
target_link_libraries(
  kinect_movement_filter
  ${catkin_LIBRARIES} )

# To check lint : catkin_make roslint_kinect_movement_filter
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -whitespace/comments")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  src/kinect_movement_filter.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
