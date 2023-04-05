project(rgbd)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
include_directories(Sophus_INCLUDE_DIRS)

add_executable(jointMap jointMap.cpp)
target_link_libraries(jointMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} fmt::fmt)
