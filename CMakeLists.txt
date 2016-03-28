cmake_minimum_required(VERSION 3.3)
project(alpha)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o0")

set(SOURCE_FILES
    Debug/a.csv
    Debug/a.exe
    Debug/b.csv
    Debug/log
    lib/lib_io.h
    lib/lib_record.h
    lib/lib_time.h
    future_net.cpp
    io.cpp
    route.cpp
    route.h)

add_executable(alpha ${SOURCE_FILES})