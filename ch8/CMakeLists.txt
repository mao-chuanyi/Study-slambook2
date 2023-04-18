cmake_minimum_required(VERSION 3.10)

project(ch8)

add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++17 ${SSE_FLAGS} -g -O3 -march=native")

include_directories("/usr/include/eigen3")
find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(fmt REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}

)

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} fmt)