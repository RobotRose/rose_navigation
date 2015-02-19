cmake_minimum_required(VERSION 2.8.3)
project(arc_local_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
            angles
            base_local_planner
            costmap_2d
            dynamic_reconfigure
            message_generation
            nav_core
            pcl_conversions
            pluginlib
            roscpp
            rose_common
            rose_conversions
            rose_geometry
            rose_twist_moving_average_filter
            rostest
            server_multiple_client
            shared_variables
            tf
            )


find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

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
    LIBRARIES arc_local_planner
    CATKIN_DEPENDS  
        angles        
        base_local_planner
        costmap_2d
        dynamic_reconfigure
        message_generation 
        nav_core
        pluginlib
        roscpp
        rose_common
        rose_geometry
        rose_conversions
        tf
    DEPENDS 
        Boost
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG


add_library(arc_local_planner 
                SHARED
                src/arc_local_planner_costmap_model.cpp
                src/arc_local_planner.cpp
                src/trajectory_footprint.cpp
                src/arc_footprint.cpp
            )

add_dependencies(arc_local_planner 
                ${catkin_EXPORTED_TARGETS} 
            )

target_link_libraries(arc_local_planner
    ${catkin_LIBRARIES}
    )


install(TARGETS
            arc_local_planner
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
