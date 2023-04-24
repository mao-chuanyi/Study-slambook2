cmake_minimum_required(VERSION 3.25)

project(ch12)

set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(fmt REQUIRED)

add_executable(mono_mapping mono_mapping.cpp)
target_link_libraries(mono_mapping ${OpenCV_LIBS} ${Sophus_LIBRARIES} fmt)

add_executable(pointcloud_mapping pointcloud_mapping.cpp)
target_link_libraries(pointcloud_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(surfel_mapping surfel_mapping.cpp)
target_link_libraries(surfel_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} fmt)

add_executable(octomap_mapping octomap_mapping.cpp)
target_link_libraries(octomap_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} fmt)
