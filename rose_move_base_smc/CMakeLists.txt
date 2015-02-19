cmake_minimum_required(VERSION 2.8.3)
project(move_base_smc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
			roscpp
            tf
            message_generation
            dynamic_reconfigure
            nav_core
            rostest
            pluginlib
            relative_positioning
            server_multiple_client
            shared_variables
            )

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS  
        roscpp
        dynamic_reconfigure
        message_generation
        tf
        nav_core      
        move_base
        relative_positioning
        shared_variables
  DEPENDS 
)



add_executable( move_base_smc
                src/move_base_smc_node.cpp
                src/move_base_smc.cpp)
add_dependencies(move_base_smc ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(move_base_smc
    ${catkin_LIBRARIES}
    )

