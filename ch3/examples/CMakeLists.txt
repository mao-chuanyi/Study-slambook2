cmake_minimum_required(VERSION 3.0)

project( examples )

include_directories( "/usr/local/include/eigen3" )

add_executable( coordinateTransform coordinateTransform.cpp)