cmake_minimum_required(VERSION 3.5)

project( ch6 )

include_directories("/usr/local/share/eigen3")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBRARIES})