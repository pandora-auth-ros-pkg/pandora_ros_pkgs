cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_victim)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_common_msgs
    pandora_vision_msgs
    state_manager
    state_manager_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    sensor_msgs
    roslint
    urdf
)

generate_dynamic_reconfigure_options(
  config/victim_dyn_reconf.config
  )

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_common_msgs
      pandora_vision_msgs
      state_manager
      state_manager_msgs
      cv_bridge
      image_transport
      dynamic_reconfigure
      sensor_msgs
      urdf
    #INCLUDE_DIRS
    #LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(victim_node
    src/victim_node.cpp
    src/victim_detection.cpp
    src/rgb_system_validator.cpp
    src/depth_system_validator.cpp
    src/victim_vj_detector.cpp

    src/utilities/channels_statistics_extractor.cpp
    src/utilities/edge_orientation_extractor.cpp
    src/utilities/haralickfeature_extractor.cpp

    src/feature_extractors/mean_std_dev.cpp
    src/feature_extractors/dominant_color.cpp
    src/feature_extractors/dft_coeffs.cpp
    src/feature_extractors/color_angles.cpp

    src/victim_parameters.cpp
)
target_link_libraries(victim_node ${catkin_LIBRARIES} )
add_dependencies(victim_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

########################### victimTrainer ############################
if(VICTIM_ENABLE_TRAIN)
  add_executable(victim_train_node
                  src/training.cpp
                  src/rgb_system_validator.cpp
                  src/depth_system_validator.cpp
                  src/utilities/channels_statistics_extractor.cpp
                  src/utilities/edge_orientation_extractor.cpp
                  src/utilities/haralickfeature_extractor.cpp

                  src/feature_extractors/mean_std_dev.cpp
                  src/feature_extractors/dominant_color.cpp
                  src/feature_extractors/dft_coeffs.cpp
                  src/feature_extractors/color_angles.cpp

                  src/victim_parameters.cpp
                  src/training_parameters.cpp
  )
  target_link_libraries(victim_train_node ${catkin_LIBRARIES} )
endif()

########################### modelTRainer ############################
if(MODEL_ENABLE_TRAIN)
  add_executable(model_train
                  src/model_trainer.cpp
  )
  target_link_libraries(model_train ${catkin_LIBRARIES} )
endif()

#~ To check lint : catkin_make roslint_pandora_vision_victim
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/victim_detection.h
  include/face_detector.h
  include/rgb_system_validator.h
  include/depth_system_validator.h
  include/utilities/channels_statistics_extractor.h
  include/utilities/edge_orientation_extractor.h
  include/utilities/haralickfeature_extractor.h
  include/training.h
  src/victim_detection.cpp
  src/victim_detector.cpp
  src/victim_vj_detector.cpp
  src/rgb_system_validator.cpp
  src/depth_system_validator.cpp
  src/utilities/channels_statistics_extractor.cpp
  src/utilities/edge_orientation_extractor.cpp
  src/utilities/haralickfeature_extractor.cpp
  src/training.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
