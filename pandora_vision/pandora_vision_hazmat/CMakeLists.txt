cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hazmat)

#~ set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    vision_communications
    state_manager
    state_manager_communications
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    roslint
    urdf
)

generate_dynamic_reconfigure_options(
  config/hazmat_cfg.cfg
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      vision_communications
      state_manager
      state_manager_communications
      cv_bridge
      image_transport
      sensor_msgs
      dynamic_reconfigure
      roslint
      urdf
    INCLUDE_DIRS
      include
    LIBRARIES 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hazmat_node 
            src/hazmat_node.cpp
						src/hazmat_detection.cpp
						src/hazmat_detector.cpp
            src/hazmat_parameters.cpp
						src/utils.cpp 
						src/sift.cpp 
						src/kdtree.cpp 
						src/minpq.cpp 
						src/xform.cpp 
						src/imgfeatures.cpp)
target_link_libraries(hazmat_node ${catkin_LIBRARIES} )
add_dependencies(hazmat_node 
    pandora_vision_hazmat_gencfg
    vision_communications_gencpp
    ${catkin_EXPORTED_TARGETS}
)

########################### hazmatTrainNode ############################
if(HAZMAT_ENABLE_TRAIN)
  add_executable(hazmat_train src/hazmat_trainer.cpp
						src/utils.cpp 
						src/sift.cpp  
						src/imgfeatures.cpp)
  target_link_libraries(hazmat_train ${catkin_LIBRARIES} )
endif()


#~ To check lint : catkin_make roslint_pandora_vision_hazmat
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  include/defines.h 
  include/hazmat_detection.h
  include/hazmat_detector.h
  src/hazmat_detector.cpp
  src/hazmat_detection.cpp
  src/hazmat_trainer.cpp
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
