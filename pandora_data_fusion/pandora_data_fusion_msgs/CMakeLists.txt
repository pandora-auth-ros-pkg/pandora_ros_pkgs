cmake_minimum_required(VERSION 2.8.3)
project(pandora_data_fusion_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation 
  std_msgs 
  actionlib_msgs 
  geometry_msgs
  visualization_msgs
  std_srvs
  )

add_message_files(
  FILES
  VictimInfoMsg.msg
  WorldModelMsg.msg
  QrNotificationMsg.msg
  GlobalProbabilitiesMsg.msg
  )

add_service_files(
  FILES
  GetObjectsSrv.srv
  DatafusionGeotiffSrv.srv
  GetMarkersSrv.srv
  )

add_action_files(
  FILES
  DeleteVictim.action
  ValidateVictim.action
  )

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs visualization_msgs std_srvs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs 
  geometry_msgs
  visualization_msgs
  std_srvs
  )
