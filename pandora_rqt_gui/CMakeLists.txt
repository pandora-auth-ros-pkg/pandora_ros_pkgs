cmake_minimum_required(VERSION 2.8.3)
project(pandora_rqt_gui)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  roslint
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
            src/pandora_rqt_gui/battery_widget.py
            src/pandora_rqt_gui/co2_widget.py
            src/pandora_rqt_gui/console.py
            src/pandora_rqt_gui/console_widget.py
            src/pandora_rqt_gui/gui_state_client.py
            src/pandora_rqt_gui/main_widget.py
            src/pandora_rqt_gui/message_data_model.py
            src/pandora_rqt_gui/pandora_gui.py
            src/pandora_rqt_gui/probability_info.py
            src/pandora_rqt_gui/sonars_widget.py
            src/pandora_rqt_gui/save_mission_client.py
            src/pandora_rqt_gui/standar_widget.py
            src/pandora_rqt_gui/temp_widget.py
            src/pandora_rqt_gui/victim_found_server.py
            src/pandora_rqt_gui/widget_info.py
            src/pandora_rqt_gui/mocks/rqt_gui.py
            test/listener.py
            test/talker.py
            test/victim_action_client.py
            test/victim_propabilities.py
            )

roslint_python(${${PROJECT_NAME}_LINT_PYTHON})


