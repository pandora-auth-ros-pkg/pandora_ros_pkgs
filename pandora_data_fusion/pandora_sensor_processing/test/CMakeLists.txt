find_package(rostest REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

########################  Tests  ####################################

########################  Unit Tests  ###############################

########################  Utils  ####################################

catkin_add_gtest(${PROJECT_NAME}_utils_test unit/utils_test.cpp)
target_link_libraries(${PROJECT_NAME}_utils_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
  gtest_main
  )

########################  Clusterer  ################################

catkin_add_gtest(${PROJECT_NAME}_clusterer_test unit/clusterer_test.cpp)
target_link_libraries(${PROJECT_NAME}_clusterer_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_clusterer
  gtest_main
  )

########################  Functional Tests  #########################

#add_rostest(functional/co2_processor_test.launch)

if(TARGET tests)
  add_executable(${PROJECT_NAME}_thermal_processor_test EXCLUDE_FROM_ALL functional/thermal_processor_test.cpp)
  target_link_libraries(${PROJECT_NAME}_thermal_processor_test
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME}_thermal_processor
    )
  add_dependencies(tests ${PROJECT_NAME}_thermal_processor_test)
endif()
#add_rostest(functional/thermal_processor_test.launch)
