cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_annotator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy 
  rosbag
  roslint
  camera_calibration_parsers
  roslib
  cv_bridge
  sensor_msgs
  pandora_vision_msgs
  pcl_ros
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 REQUIRED COMPONENTS 
  QtCore 
  QtGui 
  QtUiTools
)
catkin_python_setup()

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  DEPENDS   
    QtCore 
    QtGui 
    QtUiTools
  CATKIN_DEPENDS
    roscpp
    rospy
    rosbag
    roslib
    sensor_msgs
   pandora_vision_msgs
    cv_bridge
    pcl_ros
  INCLUDE_DIRS include
  LIBRARIES 
)

#-----------------------------------------------------------------------------#
set(QT_FORMS 
  ui/pandora_annotator.ui 
)

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

QT_WRAP_CPP(pandora_vision_annotator_node SAMPLE_MOC_SRCS 
  include/pandora_vision_annotator/annotator_connector.h
  include/pandora_vision_annotator/annotator_controller.h
  )
#-----------------------------------------------------------------------------#

add_executable(pandora_vision_annotator_node
  src/annotator_node.cpp
  src/annotator_application.cpp
  src/annotator_connector.cpp
  src/annotator_controller.cpp
  src/annotator_loader.cpp
  src/annotator_tools.cpp
  ${QT_FORMS_HPP} ${SAMPLE_MOC_SRCS}
)
add_dependencies(pandora_vision_annotator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(pandora_vision_annotator_node
  ${catkin_LIBRARIES} 
  ${QT_LIBRARIES}
)
install(PROGRAMS scripts/images_to_bag
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/split_bag
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#~ To check lint : catkin_make roslint_pandora_vision_annotator
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use")

## Additional lint filters - Not Needed ##
# -whitespace/braces, -whitespace/newline, -readability/multiline_string

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/annotator_connector.h
  include/annotator_controller.h
  include/annotator_tools.h
  src/annotator_controller.cpp
  src/annotator_loader.cpp
  src/annotator_tools.cpp
  src/annotator_node.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

