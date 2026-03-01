cmake_minimum_required(VERSION 3.10)
project(helloSLAM)

include_directories(include)

add_library(helloLib 
    src/Hello.cpp
)

add_executable(sayHello main.cpp)
target_link_libraries(sayHello helloLib)
