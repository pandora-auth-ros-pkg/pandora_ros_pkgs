cmake_minimum_required(VERSION 2.8.3)
project(pandora_alert_handler)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
          roscpp
          tf 
          actionlib 
          state_manager
          dynamic_reconfigure
          pandora_navigation_common
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

###################### objects #########################################
add_library(objects 
        src/objects.cpp)
target_link_libraries(objects 
          ${catkin_LIBRARIES}
          )
add_dependencies(objects 
  data_fusion_communications_gencpp
  ) 
          
add_library(victim 
        src/victim.cpp)
target_link_libraries(victim 
          ${catkin_LIBRARIES}
          )
add_dependencies(victim 
  data_fusion_communications_gencpp
  ) 

###################### victim_list #########################################
add_library(victim_list 
        src/victim_list.cpp)
target_link_libraries(victim_list 
          ${catkin_LIBRARIES}
          )
add_dependencies(victim_list 
  data_fusion_communications_gencpp
  )           
          
##################### Utils ############################################
add_library(utils 
        src/utils.cpp)
target_link_libraries(utils 
          ${catkin_LIBRARIES}
          )
add_dependencies(utils 
  data_fusion_communications_gencpp
  )  
  
#################### PoseFinder ########################################
add_library(pose_finder 
        src/pose_finder.cpp)
target_link_libraries(pose_finder 
          ${catkin_LIBRARIES}
          )
add_dependencies(pose_finder 
  data_fusion_communications_gencpp
  ) 
          
#################### VicitmHandler ########################################
add_library(victim_clusterer 
        src/victim_clusterer.cpp)
target_link_libraries(victim_clusterer 
          ${catkin_LIBRARIES}
          )
add_dependencies(victim_clusterer 
  data_fusion_communications_gencpp
  )           

#################### VicitmHandler ########################################
add_library(victim_handler 
        src/victim_handler.cpp)
target_link_libraries(victim_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(victim_handler 
  data_fusion_communications_gencpp
  ) 
          
##################### object_handler ############################################
add_library(object_handler 
        src/object_handler.cpp)
target_link_libraries(object_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(object_handler 
  data_fusion_communications_gencpp
  ) 

##################### alert_handler ############################################
add_library(alert_handler 
        src/alert_handler.cpp)
target_link_libraries(alert_handler 
          ${catkin_LIBRARIES}
          )
add_dependencies(alert_handler 
  pandora_alert_handler_gencfg 
  vision_communications_gencpp
  data_fusion_communications_gencpp
  ) 
add_dependencies(alert_handler 
  data_fusion_communications_gencpp
  state_manager_communications_gencpp
  ) 

##################### alert_handler_node ############################################
add_executable(alert_handler_node 
        src/main.cpp )
target_link_libraries(alert_handler_node 
          ${catkin_LIBRARIES}
          alert_handler
          object_handler
          pose_finder
          victim_handler
          victim_clusterer
          victim_list
          utils
          objects
          victim
          )
add_dependencies(alert_handler_node 
  pandora_alert_handler_gencfg 
  vision_communications_gencpp
  data_fusion_communications_gencpp
  state_manager_communications_gencpp
  ) 

include(${CMAKE_CURRENT_SOURCE_DIR}/tests.cmake)

set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,-whitespace/parens,-whitespace/comments")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} 
            include/alert_handler/*.h 
            src/*.cpp 
            )
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
