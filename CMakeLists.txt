cmake_minimum_required(VERSION 2.8)

project(HelloSLAM)


#生成一个共享库
add_library(hello_shared SHARED libHelloSLAM.cpp)

#添加一个可执行文件，并链接到刚才使用的库上
add_executable(useHello useHello.cpp)
target_link_libraries(useHello hello_shared)


