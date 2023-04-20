cmake_minimum_required(VERSION 3.25)

project(bundle_adjustment)

set(CMAKE_CXX_FLAGS "-O3 -std=c++14")

find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3"
        ${Sophus_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

add_library(bal_common common.cpp)

add_executable(bundle_adjustment_ceres bundle_adjustment_ceres.cpp)
target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common)

add_executable(bundle_adjustment_g2o bundle_adjustment_g2o.cpp)
target_link_libraries(bundle_adjustment_g2o
        g2o_core g2o_stuff g2o_types_slam3d g2o_solver_csparse
        bal_common
        fmt
        )
