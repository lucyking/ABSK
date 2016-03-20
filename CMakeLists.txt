cmake_minimum_required(VERSION 3.3)
project(codeCraft3_0_SK66_master)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

set(SOURCE_FILES
    lib/lib_io.h
    lib/lib_record.h
    lib/lib_time.h
    future_net.cpp
    io.cpp
    route.cpp
    route.h)

#add_executable(codeCraft3_0_SK66_master ${SOURCE_FILES})
add_executable(a ${SOURCE_FILES})
