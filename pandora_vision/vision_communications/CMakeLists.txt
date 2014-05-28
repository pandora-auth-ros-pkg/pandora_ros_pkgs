cmake_minimum_required(VERSION 2.8.3)
project(vision_communications)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  CandidateHoleMsg.msg
  CandidateHolesVectorMsg.msg
  HoleDirectionMsg.msg
  HolesDirectionsVectorMsg.msg
  FaceDirectionMsg.msg
  HolePositionMsg.msg
  HolesPositionsVectorMsg.msg
  EnhancedHoleMsg.msg
  EnhancedHolesVectorMsg.msg
  MotionMsg.msg
  HazmatAlertMsg.msg
  HazmatAlertsVectorMsg.msg
  QRAlertMsg.msg
  QRAlertsVectorMsg.msg
  DataMatrixAlertMsg.msg
  DataMatrixAlertsVectorMsg.msg
  LandoltcAlertMsg.msg
  LandoltcAlertsVectorMsg.msg
  LandoltcPredatorMsg.msg
  )

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
  )
