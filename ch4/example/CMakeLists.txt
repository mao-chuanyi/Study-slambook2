cmake_minimum_required(VERSION 3.0)

project( example )

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
set(FMT_LIBRARIES fmt::fmt)
add_executable(trajectoryError trajectoryError.cpp)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})
#target_link_libraries(trajectoryError PRIVATE Sophus::Sophus)