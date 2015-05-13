cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_hazmat)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_vision_common
    pandora_vision_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    sensor_msgs
    dynamic_reconfigure
    roslint
    urdf
    pandora_cmake_tools
)

generate_dynamic_reconfigure_options(
  config/pandora_vision_hazmat/Display.cfg
  config/pandora_vision_hazmat/HazmatNode.cfg
)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_vision_common
      pandora_vision_msgs
      state_manager
      state_manager_msgs
      cv_bridge
      image_transport
      sensor_msgs
      dynamic_reconfigure
      urdf
    #INCLUDE_DIRS
    LIBRARIES
      Boost
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

################# hazmat filters #################
add_library(
  ${PROJECT_NAME}_filters
  src/filters/image_signature.cpp)
target_link_libraries(
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES}
  )

################# hazmat detector #################
add_library(
  ${PROJECT_NAME}_detector
  src/detection/planar_object_detector.cpp
  )
target_link_libraries(${PROJECT_NAME}_detector
  ${PROJECT_NAME}_filters
  ${catkin_LIBRARIES}
  )
  
####### hazmat feature_matching_detector ########
add_library(
  ${PROJECT_NAME}_feature_matching_detector
  src/detection/feature_matching_detector.cpp
  )
target_link_libraries(${PROJECT_NAME}_feature_matching_detector
  ${PROJECT_NAME}_detector
  ${catkin_LIBRARIES}
  )
  
################# hazmat sift detector #################
add_library(
  ${PROJECT_NAME}_sift_detector
  src/detection/sift_detector.cpp
  )
target_link_libraries(${PROJECT_NAME}_sift_detector
  ${PROJECT_NAME}_feature_matching_detector
  ${catkin_LIBRARIES}
  )
  
################# hazmat surf detector #################
add_library(
  ${PROJECT_NAME}_surf_detector
  src/detection/surf_detector.cpp
  )
target_link_libraries(${PROJECT_NAME}_surf_detector
  ${PROJECT_NAME}_feature_matching_detector
  ${catkin_LIBRARIES}
  )

################# hazmat orb detector #################
add_library(
  ${PROJECT_NAME}_orb_detector
  src/detection/orb_detector.cpp
  )
target_link_libraries(${PROJECT_NAME}_orb_detector
  ${PROJECT_NAME}_feature_matching_detector
  ${catkin_LIBRARIES}
  )
################# hazmat detector factory #################
add_library(
  ${PROJECT_NAME}_detector_factory
  src/detection/detector_factory.cpp)
target_link_libraries(${PROJECT_NAME}_detector_factory
  ${PROJECT_NAME}_sift_detector
  ${PROJECT_NAME}_surf_detector
  ${PROJECT_NAME}_orb_detector
  ${catkin_LIBRARIES}
  )

################# hazmat processor #################
add_library(${PROJECT_NAME}_processor
      src/detection/hazmat_processor.cpp
      )
target_link_libraries(${PROJECT_NAME}_processor
    ${PROJECT_NAME}_detector_factory
    ${catkin_LIBRARIES}
    )
add_dependencies(${PROJECT_NAME}_processor
  ${PROJECT_NAME}_gencfg)


################# hazmat preprocessor #################
add_library(${PROJECT_NAME}_preprocessor
  src/detection/hazmat_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_preprocessor
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_preprocessor
  ${catkin_EXPORTED_TARGETS})

################# hazmat postprocessor #################
add_library(${PROJECT_NAME}_postprocessor
  src/detection/hazmat_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_postprocessor
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_postprocessor
  ${catkin_EXPORTED_TARGETS})

################# hazmat handler #################
add_library(${PROJECT_NAME}_handler
  src/detection/hazmat_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_processor
  ${PROJECT_NAME}_preprocessor
  ${PROJECT_NAME}_postprocessor
)

################# hazmat node #################
add_executable(${PROJECT_NAME}_node
      src/detection/hazmat_node.cpp
      )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_handler
    ${catkin_LIBRARIES}
    )
add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

############### Download necessary data to detect patterns ##################

download_extra_data(${PROJECT_NAME}_download_training_results
                    EXTRA_FILES data/extra_files.yaml)

option(${PROJECT_NAME}_training "Build flag for the training module")
########################### hazmatTrainNode ############################

if(${PROJECT_NAME}_training)
  
############### Download images for training the detector ################  
  download_extra_data(${PROJECT_NAME}_download_training_images
                    EXTRA_FILES data/training_files.yaml)

  add_library(${PROJECT_NAME}_feature_trainer
    src/training/planar_pattern_trainer.cpp 
    src/training/sift_trainer.cpp
    src/training/surf_trainer.cpp
    src/training/orb_trainer.cpp)

  target_link_libraries(${PROJECT_NAME}_feature_trainer
    ${catkin_LIBRARIES})

  add_library(${PROJECT_NAME}_trainer_factory
    src/training/factory.cpp)

  target_link_libraries(${PROJECT_NAME}_trainer_factory
    ${PROJECT_NAME}_feature_trainer
    ${catkin_LIBRARIES})

  add_executable(${PROJECT_NAME}_trainer_node 
    src/training/hazmat_trainer_node.cpp
    )
  target_link_libraries(${PROJECT_NAME}_trainer_node
    ${PROJECT_NAME}_trainer_factory
    ${catkin_LIBRARIES})

endif()


#~ To check lint : catkin_make roslint_pandora_vision_hazmat
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")

############################## Node code Lint ############################## 
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  "src/*.cpp"
  "include/*.h"
)

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

############################## Test code Lint ############################## 
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_TEST_SRC
  RELATIVE ${PROJECT_SOURCE_DIR}
  "test/*.cpp"
)
roslint_cpp(${${PROJECT_NAME}_LINT_TEST_SRC})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
