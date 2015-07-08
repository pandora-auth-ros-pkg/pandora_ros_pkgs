cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  state_manager
  state_manager_msgs
  roslint
  )

add_message_files(
  FILES ProcessorLogInfo.msg
  )

generate_messages(
  DEPENDENCIES std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    state_manager
    state_manager_msgs
  INCLUDE_DIRS
    include
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    include/**/*.h
    src/*.cpp
    )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()
