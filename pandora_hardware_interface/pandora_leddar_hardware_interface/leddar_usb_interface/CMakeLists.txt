# To compile, run: 'catkin_make -DENABLE_LEDDAR_USB=1'
# NOTE: Libraries use C++11, to compile use gcc>=4.8
if(ENABLE_LEDDAR_USB)
  # Download needed shared libraries
  download_data(
    ${PROJECT_NAME}_download_libLeddarC
    http://downloads.pandora.ee.auth.gr/data/shared_libraries/leddar/libLeddarC.so
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
    MD5 6b3ad47d9aef2c6e4c07199afebcb19c
    )
  download_data(
    ${PROJECT_NAME}_download_libLeddar
    http://downloads.pandora.ee.auth.gr/data/shared_libraries/leddar/libLeddar.so
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
    MD5 63619b147c3956c74a6dea01cc532c14
    )
  download_data(
    ${PROJECT_NAME}_download_libLeddarTech
    http://downloads.pandora.ee.auth.gr/data/shared_libraries/leddar/libLeddarTech.so
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
    MD5 afe88269a4f0767e3e58ebc976ffe578
    )

  # Add devel/lib to link directories
  link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

  add_library(
    leddar_usb_interface
    src/leddar_usb_interface.cpp
    )
  target_link_libraries(
    leddar_usb_interface
    LeddarC
    ${catkin_LIBRARIES}
    )
  add_dependencies(
    leddar_usb_interface
    ${PROJECT_NAME}_download_libLeddarC
    ${PROJECT_NAME}_download_libLeddar
    ${PROJECT_NAME}_download_libLeddarTech
    )

  add_executable(
    leddar_usb_interface_demo
    src/leddar_usb_interface_demo.cpp
    )
  target_link_libraries(
    leddar_usb_interface_demo
    leddar_usb_interface
    ${catkin_LIBRARIES}
    )
endif()
