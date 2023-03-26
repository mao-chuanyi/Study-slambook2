cmake_minimum_required(VERSION 3.0)

project( examples )

add_executable(plotTrajectory plotTrajectory.cpp)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
include_directories( "/usr/local/include/eigen3" )


add_executable( coordinateTransform coordinateTransform.cpp)

