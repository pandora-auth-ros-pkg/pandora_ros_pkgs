cmake_minimum_required(VERSION 2.8.3)
project(pandora_cmake_tools)
find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS download_data.cmake
)

catkin_install_python(PROGRAMS scripts/download_checkmd5.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
