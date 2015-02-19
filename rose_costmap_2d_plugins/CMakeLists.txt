cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d_plugins)

find_package(catkin REQUIRED
        COMPONENTS
            costmap_2d
            dynamic_reconfigure
            geometry_msgs
            pluginlib
            roscpp
            rose_common
            rose_conversions
            std_msgs
            tf
        )

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package( 
    INCLUDE_DIRS
        include
    LIBRARIES costmap_2d_plugins
    CATKIN_DEPENDS
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        pluginlib
        roscpp
        rose_common
        rose_conversions
        std_msgs
        tf
    DEPENDS
        Boost
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

add_library(costmap_2d_plugins
    src/simulation_obstacle_layer.cpp
)

target_link_libraries(costmap_2d_plugins
    ${catkin_LIBRARIES}
    )


install(TARGETS
            costmap_2d_plugins
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
