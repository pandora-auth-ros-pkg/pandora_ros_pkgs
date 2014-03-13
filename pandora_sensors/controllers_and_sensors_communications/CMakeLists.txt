# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(controllers_and_sensors_communications)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  actionlib_msgs
  std_msgs
)



#uncomment if you have defined messages
add_message_files(
  FILES
  co2Msg.msg
  mlxTempMsg.msg
  headIrMsg.msg
  headSonarMsg.msg
  sonarMsg.msg
  compassMsg.msg
  butterflyMsg.msg
  soundExistenceMsg.msg
  irMsg.msg
  tpaMsg.msg
  )

add_service_files(
  FILES
  controlSwitchSrv.srv
  gripperControlSrv.srv
  tiltControlSrv.srv
  )
  
add_action_files(
  FILES
  controlServo.action
  )


generate_messages(
  DEPENDENCIES 
    actionlib_msgs
    std_msgs
)


catkin_package(
  CATKIN_DEPENDS 
    message_generation
    message_runtime
    std_msgs
    actionlib_msgs
)
