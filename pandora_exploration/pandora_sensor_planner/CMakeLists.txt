cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_planner)

################################################################################
#                                 Find Package                                 #
################################################################################
# - Use `find_package` command to bring dependencies
# - Use `generate_dynamic_reconfigure_options` to setup dynamic reconfigure
# - Set flags if needed

################################################################################
#                             Catkin Package Setup                             #
################################################################################
# - Use `catkin_package` to declare this catkin package and how it interacts
#   with other packages in the catkin package ecosystem
# - Use `catkin_python_setup` to setup catkin python ecosystem with ./setup.py
# - Set `include_directories`
# - Set `link_directories` (if necessary, usually avoid to if concerns catkin
#   libraries)

################################################################################
#                                  Libraries                                   #
################################################################################
# - Declare software libraries that will be made in this package and their
#   target names with `add_library`
# - Set `target_link_libraries` for each library target
# - Set `add_dependencies`, if necessary, to `${catkin_EXPORTED_TARGETS}` or
#   others

################################################################################
#                                 Executables                                  #
################################################################################
# - Declare software executables that will be made in this package and their
#   target names with `add_executable`
# - Set `target_link_libraries` for each executable target

################################################################################
#                                   Roslint
################################################################################
# - Setup roslint for test code with `roslint_cpp`, `roslint_python` or
# `roslint_custom`

################################################################################
#                                   Testing                                    #
################################################################################
# - Declare test subdirectory, in order not to pollute source code's
#   CMakeLists.txt with it

if(CATKIN_ENABLE_TESTING)
  # add_subdirectory(test)
endif()

################################################################################
#                               Install Package                                #
################################################################################
# - Install directory of header files with `install(DIRECTORY ...)`
# - Install targets of exported libraries with `install(TARGETS ...)`
# - Install targets of executables with `install(TARGETS ...)`
