cmake_minimum_required(VERSION 2.8.3)
project(pandora_rqt_gui)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
)

catkin_python_setup()

add_action_files(
  FILES
  ValidateVictimGUI.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_mypkg
  CATKIN_DEPENDS
    rospy
    rqt_gui
    rqt_gui_py
    message_runtime
    message_generation
    std_msgs
    actionlib_msgs
    pandora_data_fusion_msgs
#  DEPENDS system_lib
)
