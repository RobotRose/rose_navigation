cmake_minimum_required(VERSION 2.8.3)
project(rose_move_base_smc)

find_package(catkin REQUIRED COMPONENTS
			roscpp
            tf
            message_generation
            dynamic_reconfigure
            nav_core
            rostest
            pluginlib
            rose_relative_positioning
            server_multiple_client
            rose_shared_variables
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
  CATKIN_DEPENDS  
    roscpp
    dynamic_reconfigure
    message_generation
    tf
    nav_core      
    move_base
    rose_relative_positioning
    rose_shared_variables
  DEPENDS 
)

add_executable( rose_move_base_smc
                src/move_base_smc_node.cpp
                src/move_base_smc.cpp
)

add_dependencies(rose_move_base_smc ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(rose_move_base_smc
    ${catkin_LIBRARIES}
)
