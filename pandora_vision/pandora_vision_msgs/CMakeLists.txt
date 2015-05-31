cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_msgs)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  #actionlib_msgs
  pandora_common_msgs
)

add_message_files(
  FILES
  Blob.msg
  BlobVector.msg
  DataMatrixAlert.msg
  DataMatrixAlertVector.msg
  EnhancedImage.msg
  HazmatAlert.msg
  HazmatAlertVector.msg
  HoleDirectionAlert.msg
  HoleDirectionAlertVector.msg
  HolePositionAlert.msg
  HolePositionAlertVector.msg
  Keypoint.msg
  LandoltcAlert.msg
  LandoltcAlertVector.msg
  Predator.msg
  QRAlert.msg
  QRAlertVector.msg
  RegionOfInterest.msg
  RegionOfInterestVector.msg
  SynchronizedMsg.msg
  )

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    pandora_common_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
  )
