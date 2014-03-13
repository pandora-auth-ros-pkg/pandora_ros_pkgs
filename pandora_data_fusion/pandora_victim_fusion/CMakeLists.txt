cmake_minimum_required(VERSION 2.8.3)
project(pandora_victim_fusion)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
          roscpp
          state_manager
          data_fusion_communications 
          controllers_and_sensors_communications 
          vision_communications 
          )

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(include ${catkin_INCLUDE_DIRS})

#---------------Victim fusion node--------------------
add_executable(victim_fusion src/main.cpp
                             src/co2.cpp 
                             src/camera.cpp 
                             src/mlx.cpp 
                             src/tpa.cpp

                             src/state_abstract.cpp 
                             src/state_arm_approach.cpp 
                             src/state_df_hold.cpp
                             src/state_exploration_identification.cpp 
                             src/state_any_other.cpp

                             src/victim_fusion.cpp 
          )
target_link_libraries(victim_fusion ${catkin_LIBRARIES} )
add_dependencies(victim_fusion 
  controllers_and_sensors_communications_gencpp
  vision_communications_gencpp
  data_fusion_communications_gencpp
  state_manager_communications_gencpp
  ) 

