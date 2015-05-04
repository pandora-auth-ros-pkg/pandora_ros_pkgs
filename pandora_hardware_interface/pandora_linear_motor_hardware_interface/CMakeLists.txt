cmake_minimum_required(VERSION 2.8.3)
project(pandora_linear_motor_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
)

set(THIS_PACKAGE_INCLUDE_DIRS
    linear_motor_com_interface/include
    linear_motor_hardware_interface/include
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    controller_manager
    controller_interface
    serial
    
  LIBRARIES
  DEPENDS
)


#--------------------------------Find libusb------------------------------------
if (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
    # in cache already
    set(LIBUSB_FOUND TRUE)
else (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  find_path(LIBUSB_1_INCLUDE_DIR
    NAMES
      libusb-1.0/libusb.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
      PATH_SUFFIXES
      libusb-1.0
)
find_library(LIBUSB_1_LIBRARY
  NAMES
    usb-1.0
  PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
)
set(LIBUSB_1_INCLUDE_DIRS
  ${LIBUSB_1_INCLUDE_DIR}
)
set(LIBUSB_1_LIBRARIES
  ${LIBUSB_1_LIBRARY}
)
if (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)
  set(LIBUSB_1_FOUND TRUE)
endif (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)
if (LIBUSB_1_FOUND)
  if (NOT libusb_1_FIND_QUIETLY)
    message(STATUS "Found libusb-1.0:")
    message(STATUS " - Includes: ${LIBUSB_1_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${LIBUSB_1_LIBRARIES}")
  endif (NOT libusb_1_FIND_QUIETLY)
  else (LIBUSB_1_FOUND)
    if (libusb_1_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find libusb")
    endif (libusb_1_FIND_REQUIRED)
  endif (LIBUSB_1_FOUND)
# show the LIBUSB_1_INCLUDE_DIRS and LIBUSB_1_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBUSB_1_INCLUDE_DIRS LIBUSB_1_LIBRARIES)
endif (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
#-------------------------------------------------------------------------------

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${LIBUSB_1_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(linear_motor_com_interface)
add_subdirectory(linear_motor_hardware_interface)

# To check lint : catkin_make roslint_pandora_linear_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")
  
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  linear_motor_com_interface/*.h
  linear_motor_com_interface/*.cpp
  linear_motor_hardware_interface/*.h 
  linear_motor_hardware_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
