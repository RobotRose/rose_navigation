cmake_minimum_required(VERSION 2.8.3)
project(rose_footprint_collision_checker)

find_package(catkin 
	REQUIRED 
	COMPONENTS
        ros_clipper
        roscpp
        rose_conversions
        rose_generic_platform
        rose_geometry
        rose_transformations
    )


find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
        rose_footprint_collision_checker 
    CATKIN_DEPENDS 
        ros_clipper 
        rose_conversions
        rose_generic_platform
        rose_geometry
        rose_transformations
    DEPENDS 
        polyclipping
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} )

# Footprint collision checker lib 
add_library(rose_footprint_collision_checker SHARED  
    src/footprint_collision_checker.cpp
)

target_link_libraries(rose_footprint_collision_checker ${catkin_LIBRARIES} 
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-utest 
    test/footprint_collision_checker_utest.cpp
)

if(TARGET ${PROJECT_NAME}-utest)
  target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
