cmake_minimum_required(VERSION 3.0)

project( useSophus )

find_package(Sophus REQUIRED)

include_directories("/usr/local/include/eigen3")

add_executable(useSophus useSophus.cpp )

#include_directories(${Sophus_INCLUDE_DIRS})
#target_link_libraries(useSophus ${Sophus_LIBRARIES})
target_link_libraries(useSophus PRIVATE Sophus::Sophus)





#add_subdirectory(example)