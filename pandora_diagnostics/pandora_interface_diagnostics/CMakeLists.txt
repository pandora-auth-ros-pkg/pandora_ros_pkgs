cmake_minimum_required(VERSION 2.8.3)
project(pandora_interface_diagnostics)
find_package(catkin REQUIRED COMPONENTS 
            roscpp 
            state_manager 
            state_manager_msgs 
            diagnostic_updater 
            interface_tester
            cmake_modules
            )
            
find_package(TinyXML REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

catkin_package(
    DEPENDS TinyXML
    CATKIN-DEPENDS roscpp 
                   state_manager 
                   state_manager_msgs 
                   diagnostic_updater 
                   interface_tester
    INCLUDE_DIRS 
    LIBRARIES 
)

add_library(generic_diagnostic src/generic_diagnostic.cpp)
target_link_libraries(generic_diagnostic   
      ${catkin_LIBRARIES}
      )
      
add_library(interfaces_xml_parser src/interfaces_xml_parser.cpp)
target_link_libraries(interfaces_xml_parser   
      ${catkin_LIBRARIES}
      )

add_library(node_diagnostics src/node_diagnostics.cpp)
target_link_libraries(node_diagnostics   
      ${catkin_LIBRARIES}
      ${TinyXML_LIBRARIES}
      generic_diagnostic
      interfaces_xml_parser
      )
add_dependencies(node_diagnostics state_manager_msgs_gencpp)

add_library(interface_diagnostics src/interface_diagnostics.cpp)
target_link_libraries(interface_diagnostics   
      ${catkin_LIBRARIES}
      ${TinyXML_LIBRARIES}
      generic_diagnostic
      interfaces_xml_parser
      )
add_dependencies(interface_diagnostics state_manager_msgs_gencpp)

add_executable(interface_diagnostics_node src/interface_diagnostics_node.cpp)
target_link_libraries(interface_diagnostics_node   
      ${catkin_LIBRARIES}
      ${TinyXML_LIBRARIES}
      node_diagnostics
      interface_diagnostics
      generic_diagnostic
      interfaces_xml_parser
      )
add_dependencies(interface_diagnostics_node state_manager_msgs_gencpp)

