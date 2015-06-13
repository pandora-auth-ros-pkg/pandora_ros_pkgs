cmake_minimum_required(VERSION 2.8.3)
project(pandora_rqt_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  roslint
  )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    rqt_gui
    rqt_gui_py
    message_runtime
    pandora_data_fusion_msgs
    pandora_geotiff
    pandora_gui_msgs
    pandora_sensor_msgs
    state_manager_msgs
  )

################ RosLint ###################################################
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
  ${PROJECT_SOURCE_DIR}
  src/**/*.py
  test/*.py
  )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})
