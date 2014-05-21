cmake_minimum_required(VERSION 2.8.3)
project(pandora_vision_predator)
set(CMAKE_BUILD_TYPE Release)



find_package(catkin REQUIRED COMPONENTS
cv_bridge
vision_communications
roscpp
image_transport
sensor_msgs
roslint
)


find_package(OpenCV REQUIRED)


include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(
src/libopentld/mftracker
src/libopentld/tld
src/3rdparty/cvblobs)



catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
 roscpp
 cv_bridge
 image_transport
 vision_communications
 sensor_msgs
 roslint
 DEPENDS system_lib
)



add_library(cvblobs STATIC
src/3rdparty/cvblobs/blob.cpp
src/3rdparty/cvblobs/BlobContour.cpp
src/3rdparty/cvblobs/BlobOperators.cpp
src/3rdparty/cvblobs/BlobProperties.cpp
src/3rdparty/cvblobs/BlobResult.cpp
src/3rdparty/cvblobs/ComponentLabeling.cpp
src/3rdparty/cvblobs/blob.h
src/3rdparty/cvblobs/BlobContour.h
src/3rdparty/cvblobs/BlobLibraryConfiguration.h
src/3rdparty/cvblobs/BlobOperators.h
src/3rdparty/cvblobs/BlobProperties.h
src/3rdparty/cvblobs/BlobResult.h
src/3rdparty/cvblobs/ComponentLabeling.h)


include_directories(
    src/libopentld/mftracker
    src/libopentld/tld
    src/3rdparty/cvblobs
    ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})


add_library(libopentld STATIC
src/libopentld/mftracker/BB.cpp
src/libopentld/mftracker/BBPredict.cpp
src/libopentld/mftracker/FBTrack.cpp
src/libopentld/mftracker/Lk.cpp
src/libopentld/mftracker/Median.cpp
src/libopentld/tld/Clustering.cpp
src/libopentld/tld/DetectionResult.cpp
src/libopentld/tld/DetectorCascade.cpp
src/libopentld/tld/EnsembleClassifier.cpp
src/libopentld/tld/ForegroundDetector.cpp
src/libopentld/tld/MedianFlowTracker.cpp
src/libopentld/tld/NNClassifier.cpp
src/libopentld/tld/TLD.cpp
src/libopentld/tld/TLDUtil.cpp
src/libopentld/tld/VarianceFilter.cpp
src/libopentld/mftracker/BB.h
src/libopentld/mftracker/BBPredict.h
src/libopentld/mftracker/FBTrack.h
src/libopentld/mftracker/Lk.h
src/libopentld/mftracker/Median.h
src/libopentld/tld/Clustering.h
src/libopentld/tld/DetectionResult.h
src/libopentld/tld/DetectorCascade.h
src/libopentld/tld/EnsembleClassifier.h
src/libopentld/tld/ForegroundDetector.h
src/libopentld/tld/IntegralImage.h
src/libopentld/tld/MedianFlowTracker.h
src/libopentld/tld/NNClassifier.h
src/libopentld/tld/NormalizedPatch.h
src/libopentld/tld/TLD.h
src/libopentld/tld/TLDUtil.h
src/libopentld/tld/VarianceFilter.h)
    


target_link_libraries(libopentld ${OpenCV_LIBS})

set_target_properties(libopentld PROPERTIES OUTPUT_NAME opentld)


link_directories(${OpenCV_LIB_DIR})

include_directories(
        src/libopentld/mftracker
        src/libopentld/tld
        src/3rdparty/cvblobs
    ${OpenCV_INCLUDE_DIRS})



target_link_libraries(libopentld cvblobs ${OpenCV_LIBS})

add_executable(predator_node src/predator_node.cpp)

target_link_libraries(predator_node cvblobs libopentld ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_dependencies(predator_node 
    ${catkin_EXPORTED_TARGETS}
    vision_communications_gencpp
    
)

#~ To check lint : catkin_make roslint_pandora_vision_predator
set(ROSLINT_CPP_OPTS 
    "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string")
    
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
  RELATIVE ${PROJECT_SOURCE_DIR} 
            
  src/predator_node.cpp
  include/predator_node.h
  
            
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
