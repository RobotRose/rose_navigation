cmake_minimum_required(VERSION 2.8.3)
project(rose_arc_local_planner)

find_package(catkin REQUIRED COMPONENTS
            angles
            base_local_planner
            costmap_2d
            dynamic_reconfigure
            geometry_msgs
            message_generation
            nav_core
            nav_msgs
            pluginlib
            roscpp
            rose_common
            rose_conversions
            rose_geometry
            rose_twist_moving_average_filter
            rostest
            server_multiple_client
            rose_shared_variables
            std_msgs
            tf
            visualization_msgs
            )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES 
        rose_arc_local_planner
    CATKIN_DEPENDS  
        angles
        base_local_planner
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        nav_core
        nav_msgs
        pluginlib
        roscpp
        rose_common
        rose_conversions
        rose_geometry
        rose_twist_moving_average_filter
        rostest
        server_multiple_client
        rose_shared_variables
        std_msgs
        tf
        visualization_msgs
    DEPENDS 
        Boost
        eigen
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(rose_arc_local_planner 
                SHARED
                src/arc_local_planner_costmap_model.cpp
                src/arc_local_planner.cpp
                src/trajectory_footprint.cpp
                src/arc_footprint.cpp
            )

add_dependencies(rose_arc_local_planner 
                ${catkin_EXPORTED_TARGETS} 
            )

target_link_libraries(rose_arc_local_planner
    ${catkin_LIBRARIES}
    )


install(TARGETS
            rose_arc_local_planner
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
