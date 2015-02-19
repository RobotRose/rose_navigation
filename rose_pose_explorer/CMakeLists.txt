cmake_minimum_required(VERSION 2.8.3)
project(rose_pose_explorer)

find_package(catkin REQUIRED
	COMPONENTS
	genmsg
	geometry_msgs
	message_generation
	roscpp
	rose_common
	rose_geometry
	sensor_msgs
	std_msgs
	tf
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  reachable_poses.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
 	INCLUDE_DIRS 
 		include
	LIBRARIES
	CATKIN_DEPENDS 
		geometry_msgs
		message_runtime
		rose_common
		rose_geometry
		sensor_msgs
		roscpp
		genmsg
		std_msgs
		tf
	DEPENDS
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_pose_explorer 	src/pose_explorer_node.cpp
								src/pose_explorer.cpp
)

target_link_libraries( rose_pose_explorer ${catkin_LIBRARIES} )
add_dependencies( rose_pose_explorer ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
