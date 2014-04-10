cmake_minimum_required(VERSION 2.8.3)
project(pandora_alert_handler)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
          roscpp
          tf 
          actionlib 
          state_manager
          dynamic_reconfigure
          roslint
          data_fusion_communications 
          vision_communications 
          )

generate_dynamic_reconfigure_options(
  config/alert_handler/alert_handler_dynamic.cfg
  config/alert_handler/mass_alert_publisher_dymamic.cfg
  )

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(include ${catkin_INCLUDE_DIRS})

          
##################### utils ############################################

add_library(utils 
        src/utils.cpp)
target_link_libraries(utils 
          ${catkin_LIBRARIES}
          )
add_dependencies(utils 
  data_fusion_communications_gencpp
  )

###################### objects #########################################

add_library(objects 
        src/objects.cpp)
target_link_libraries(objects
          ${catkin_LIBRARIES}
          utils
          )
add_dependencies(objects 
  data_fusion_communications_gencpp
  ) 

add_library(hazmat 
        src/hazmat.cpp)
target_link_libraries(hazmat
          ${catkin_LIBRARIES}
          objects
          )
add_dependencies(hazmat
  data_fusion_communications_gencpp
  ) 

add_library(hole
        src/hole.cpp)
target_link_libraries(hole
          ${catkin_LIBRARIES}
          objects
          )
add_dependencies(hole
  data_fusion_communications_gencpp
  ) 

add_library(qr 
        src/qr.cpp)
target_link_libraries(qr 
          ${catkin_LIBRARIES}
          objects
          )
add_dependencies(qr 
  data_fusion_communications_gencpp
  ) 

add_library(tpa 
        src/tpa.cpp)
target_link_libraries(tpa 
          ${catkin_LIBRARIES}
          objects
          )
add_dependencies(tpa
  data_fusion_communications_gencpp
  ) 

add_library(victim 
        src/victim.cpp)
target_link_libraries(victim
          ${catkin_LIBRARIES}
          objects 
          )
add_dependencies(victim 
  data_fusion_communications_gencpp
  ) 

###################### victim_list #####################################

add_library(victim_list 
        src/victim_list.cpp)
target_link_libraries(victim_list
          ${catkin_LIBRARIES}
          utils
          victim
          )
add_dependencies(victim_list 
  data_fusion_communications_gencpp
  )           

#################### ros_tf_listener ####################################

add_library(ros_tf_listener 
        src/ros_tf_listener.cpp)
target_link_libraries(ros_tf_listener 
          ${catkin_LIBRARIES}
          )
add_dependencies(ros_tf_listener
  data_fusion_communications_gencpp
  ) 

#################### pose_finder ########################################

add_library(pose_finder 
        src/pose_finder.cpp)
target_link_libraries(pose_finder
          ${catkin_LIBRARIES}
          ros_tf_listener
          objects
          utils
          )
add_dependencies(pose_finder 
  data_fusion_communications_gencpp
  ) 

#################### object_factory ###################################

add_library(object_factory 
        src/object_factory.cpp)
target_link_libraries(object_factory
          ${catkin_LIBRARIES}
          pose_finder
          hazmat
          hole
          qr
          tpa
          utils
          )
add_dependencies(object_factory 
  data_fusion_communications_gencpp
  vision_communications_gencpp
  ) 
         
#################### victim_clusterer ###################################

add_library(victim_clusterer 
        src/victim_clusterer.cpp)
target_link_libraries(victim_clusterer 
          ${catkin_LIBRARIES}
          victim
          )
add_dependencies(victim_clusterer 
  data_fusion_communications_gencpp
  )           

#################### victim_handler #####################################

add_library(victim_handler 
        src/victim_handler.cpp)
target_link_libraries(victim_handler
          ${catkin_LIBRARIES}
          victim
          victim_clusterer
          victim_list
          )
add_dependencies(victim_handler 
  data_fusion_communications_gencpp
  ) 
          
##################### object_handler ####################################

add_library(object_handler 
        src/object_handler.cpp)
target_link_libraries(object_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(object_handler 
  data_fusion_communications_gencpp
  ) 

##################### alert_handler #####################################

add_library(alert_handler 
        src/alert_handler.cpp)
target_link_libraries(alert_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(alert_handler 
  pandora_alert_handler_gencfg 
  vision_communications_gencpp
  data_fusion_communications_gencpp
  state_manager_communications_gencpp
  ) 

##################### alert_handler_node ###############################

add_executable(alert_handler_node 
        src/main.cpp )
target_link_libraries(alert_handler_node 
          ${catkin_LIBRARIES}
          alert_handler
          object_factory
          object_handler
          pose_finder
          ros_tf_listener
          victim_handler
          victim_clusterer
          victim_list
          utils
          objects
          hazmat
          hole
          qr
          tpa
          victim
          )
add_dependencies(alert_handler_node 
  pandora_alert_handler_gencfg 
  vision_communications_gencpp
  data_fusion_communications_gencpp
  state_manager_communications_gencpp
  ) 

################ RosLint ###################################################

set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include,-whitespace/blank_line,-whitespace/parens, -whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            include/alert_handler/*.h 
            src/*.cpp 
            )
LIST(REMOVE_ITEM ${PROJECT_NAME}_LINT_SRCS
      RELATIVE ${PROJECT_SOURCE_DIR}
            include/alert_handler/alert_handler.h
            include/alert_handler/tf_listener.h
            src/alert_handler.cpp
            src/ros_tf_listener.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include,-whitespace/blank_line,-whitespace/parens, -whitespace/braces, -runtime/references")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS_WITH_REFS
     RELATIVE ${PROJECT_SOURCE_DIR} 
            include/alert_handler/alert_handler.h
            include/alert_handler/tf_listener.h
            src/alert_handler.cpp
            src/ros_tf_listener.cpp
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS_WITH_REFS})

####### Testing ####

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

