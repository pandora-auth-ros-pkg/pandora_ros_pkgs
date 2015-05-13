cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_victim)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pandora_common_msgs
    pandora_vision_msgs
    pandora_vision_common
    sensor_processor
    state_manager
    state_manager_msgs
    pandora_cmake_tools
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
      pandora_vision_common
      sensor_processor
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

############################### Utilities Library ##############################
add_library(${PROJECT_NAME}_utilities
    src/utilities/file_utilities.cpp
    src/utilities/feature_extraction_utilities.cpp
    src/utilities/bag_of_words_trainer.cpp
)
target_link_libraries(${PROJECT_NAME}_utilities
    ${catkin_LIBRARIES}
)

######################## Victim_Parameters library #############################

add_library(${PROJECT_NAME}_victim_parameters
    src/victim_parameters.cpp
)
add_dependencies(${PROJECT_NAME}_victim_parameters
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_victim_parameters
    ${catkin_LIBRARIES}
)

################ Channels Statistics Feature Extractors library ################
add_library(${PROJECT_NAME}_channels_statistics_feature_extractors
    src/channels_statistics_feature_extractors/mean_std_dev_extractor.cpp
    src/channels_statistics_feature_extractors/dominant_color_extractor.cpp
    src/channels_statistics_feature_extractors/dft_coeffs_extractor.cpp
    src/channels_statistics_feature_extractors/color_angles_extractor.cpp
)

target_link_libraries(${PROJECT_NAME}_channels_statistics_feature_extractors
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_victim_parameters
)

########################## Feature Extractors library ##########################
add_library(${PROJECT_NAME}_feature_extractors
    src/feature_extractors/channels_statistics_extractor.cpp
    src/feature_extractors/edge_orientation_extractor.cpp
    src/feature_extractors/haralickfeature_extractor.cpp
    src/feature_extractors/sift_extractor.cpp
    src/feature_extractors/hog_extractor.cpp
    src/feature_extractors/feature_extraction.cpp
    src/feature_extractors/depth_feature_extraction.cpp
    src/feature_extractors/rgb_feature_extraction.cpp
    src/feature_extractors/histogram_extractor.cpp
)

target_link_libraries(${PROJECT_NAME}_feature_extractors
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_channels_statistics_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    ${PROJECT_NAME}_utilities
)

########################## SVM Classifier Library ##############################
add_library(${PROJECT_NAME}_svm_classifier
    src/svm_classifier/svm_training.cpp
    src/svm_classifier/rgb_svm_training.cpp
    src/svm_classifier/depth_svm_training.cpp
)

target_link_libraries(${PROJECT_NAME}_svm_classifier
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_feature_extractors
    ${PROJECT_NAME}_victim_parameters
    ${PROJECT_NAME}_utilities
)


################# victim hole processor #################
add_library(${PROJECT_NAME}_hole_processor
  src/victim_hole_processor.cpp
  src/svm_classifier/svm_validator.cpp
  src/svm_classifier/rgb_svm_validator.cpp
  src/svm_classifier/depth_svm_validator.cpp
)
target_link_libraries(${PROJECT_NAME}_hole_processor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim_parameters
  ${PROJECT_NAME}_feature_extractors
)

################# victim hole preprocessor #################
add_library(${PROJECT_NAME}_hole_preprocessor
  src/victim_hole_preprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_hole_preprocessor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_victim_parameters
)

add_dependencies(${PROJECT_NAME}_hole_preprocessor
  ${catkin_EXPORTED_TARGETS})
################# victim image processor #################
#~ add_library(${PROJECT_NAME}_image_processor
  #~ src/victim_image_processor.cpp
#~ )
#~ target_link_libraries(${PROJECT_NAME}_image_processor
  #~ ${catkin_LIBRARIES}
  #~ ${PROJECT_NAME}_victim_parameters
#~ )

################# victim image preprocessor #################
#~ add_library(${PROJECT_NAME}_image_preprocessor
  #~ src/victim_image_preprocessor.cpp
#~ )
#~ target_link_libraries(${PROJECT_NAME}_image_preprocessor
  #~ ${catkin_LIBRARIES}
#~ )

################# victim postprocessor #################
add_library(${PROJECT_NAME}_postprocessor
  src/victim_postprocessor.cpp
)
target_link_libraries(${PROJECT_NAME}_postprocessor
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_postprocessor
  ${catkin_EXPORTED_TARGETS})
################# victim handler #################
add_library(${PROJECT_NAME}_handler
  src/victim_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_hole_processor
  ${PROJECT_NAME}_hole_preprocessor
  #~ ${PROJECT_NAME}_image_processor
  #~ ${PROJECT_NAME}_image_preprocessor
  ${PROJECT_NAME}_postprocessor
)

################# victim node #################
add_executable(${PROJECT_NAME}_node
  src/victim_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_handler
)


############################## Victim SVM Trainer ##############################
if(VICTIM_ENABLE_TRAIN)

  add_executable(victim_rgb_svm_training_node
      src/svm_classifier/victim_rgb_svm_training_node.cpp
  )

  target_link_libraries(victim_rgb_svm_training_node
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_svm_classifier
  )

  add_dependencies(victim_rgb_svm_training_node
      ${catkin_EXPORTED_TARGETS}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

  add_executable(victim_depth_svm_training_node
      src/svm_classifier/victim_depth_svm_training_node.cpp
  )

  target_link_libraries(victim_depth_svm_training_node
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_svm_classifier
  )

  add_dependencies(victim_depth_svm_training_node
      ${catkin_EXPORTED_TARGETS}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
endif()

################################ Model Trainer #################################
if(MODEL_ENABLE_TRAIN)
  add_executable(model_train
      src/model_trainer.cpp
  )
  target_link_libraries(model_train
      ${catkin_LIBRARIES}
  )
endif()

# download_extra_data(
  # ${PROJECT_NAME}_download_training_files
  # EXTRA_FILES data/extra_files.yaml
  # )

#~ To check lint : catkin_make roslint_pandora_vision_victim
set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use")
    
## Additional lint filters - Not Needed ##
# -whitespace/braces, -whitespace/newline, -readability/multiline_string

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  src/*.cpp
  test/*.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})

##################################### Testing ##################################

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
