cmake_minimum_required(VERSION 2.8.3)
project(pandora_bounds_diagnostics)
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    tf
                    std_msgs
                    )

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Conrollers has changed, so we need to write new bound checks.
## controllers_and_sensors_communications package no longer exists.
#~ add_executable  (boundsCheck
                #~ src/bounds_check.cpp
                #~
                #~ src/controllers_and_sensors/overall_controllers_checker.cpp
                #~ src/controllers_and_sensors/butterfly_checker.cpp
                #~ src/controllers_and_sensors/co2_checker.cpp
                #~ src/controllers_and_sensors/compass_checker.cpp
                #~ src/controllers_and_sensors/irs_checker.cpp
                #~ src/controllers_and_sensors/mlx_checker.cpp
                #~ src/controllers_and_sensors/sonars_checker.cpp
                #~ src/controllers_and_sensors/tpa_checker.cpp
                #~ )
#~ target_link_libraries (boundsCheck ${catkin_LIBRARIES} )
#~ add_dependencies(boundsCheck controllers_and_sensors_communications_gencpp)
