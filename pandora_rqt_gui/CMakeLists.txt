cmake_minimum_required(VERSION 2.8.3)
project(pandora_rqt_gui)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  roslint
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    rqt_gui
    rqt_gui_py
    message_runtime
    message_generation
    std_msgs
    actionlib_msgs
    pandora_data_fusion_msgs
    roslint
)

################ RosLint ###################################################
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
            ${PROJECT_SOURCE_DIR}
            src/**/*.py
            test/*.py
            )

roslint_python(${${PROJECT_NAME}_LINT_PYTHON})


