add_library(${PROJECT_NAME}_thermal
  thermal.cpp
  hole_detector.cpp
  temperatures.cpp
  temperatures_parameters.cpp
  )

add_library(${PROJECT_NAME}_thermalcropper
  thermal_cropper.cpp
  )

target_link_libraries(${PROJECT_NAME}_thermal
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_thermalcropper
  ${PROJECT_NAME}_thermal
  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_thermal
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(thermal_camera_node thermal_node.cpp)
add_executable(thermal_temperatures_node temperatures_node.cpp)
add_executable(thermal_cropper_node thermal_cropper_node.cpp)

target_link_libraries(thermal_camera_node
  ${PROJECT_NAME}_thermal
  ${catkin_LIBRARIES})

target_link_libraries(thermal_temperatures_node
  ${PROJECT_NAME}_thermal
  ${catkin_LIBRARIES})

target_link_libraries(thermal_cropper_node
  ${PROJECT_NAME}_thermalcropper
  ${catkin_LIBRARIES})
