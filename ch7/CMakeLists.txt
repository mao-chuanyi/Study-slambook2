cmake_minimum_required(VERSION 3.25)

project(ch7)

add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories("/usr/local/include/eigen3/")
find_package(OpenCV 4 REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
include_directories(
        ${G2O_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        )

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        ${Sophus_LIBS}
        fmt
        )

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        fmt
        )