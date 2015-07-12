cmake_minimum_required(VERSION 2.8.3)
project(pandora_motion_control)

################################################################################
#                          Find Package Dependencies                           #
################################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pandora_motor_hardware_interface
  message_generation
  roslint
)

find_package(Boost REQUIRED COMPONENTS thread)

################################################################################
#                              Declare Messages                                #
################################################################################

# Declare Kinodynamic Command ? // Paizei na exei metafer8ei sto HW Interface

add_service_files(
  FILES
  StoreAVTable.srv
)

catkin_python_setup()
generate_messages()
################################################################################
#                               Declare Package                                #
################################################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    pandora_motor_hardware_interface
    message_generation
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

################################################################################
#                                Build Targets                                 #
################################################################################

################################################################################
#                          Pandora Kinodynamic Model                           #
################################################################################

################################################################################
#                         Pandora Kinodynamic Control                          #
################################################################################

################################################################################
#                                   Roslint                                    #
################################################################################

################################################################################
#                                   Testing                                    #
################################################################################

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()

################################################################################
#                               Install Targets                                #
################################################################################
