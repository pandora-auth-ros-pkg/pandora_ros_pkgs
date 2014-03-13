cmake_minimum_required(VERSION 2.8.3)
project(state_manager_communications)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs actionlib_msgs)

add_message_files(
	FILES
	robotModeMsg.msg
	StateMonitorUpdateMsg.msg
	)

add_service_files(
	FILES
	registerNodeSrv.srv
	)
	
add_action_files(
	FILES
	RobotMode.action
	)

generate_messages(
	DEPENDENCIES std_msgs actionlib_msgs
)
catkin_package(
	CATKIN_DEPENDS message_runtime 
)
