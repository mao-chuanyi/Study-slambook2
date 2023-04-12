cmake_minimum_required(VERSION 3.25)

project( ch6 )


set(CMAKE_CXX_FLAGS "-std=c++17")
#eigen3
include_directories("/usr/local/include/eigen3")
#openCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
#g20
SET( G2O_ROOT /usr/local/include/g2o )
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBRARIES})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o_core g2o_stuff glog)