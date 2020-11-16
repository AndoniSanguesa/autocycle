cmake_minimum_required(VERSION 3.17)
project(LiDAR)

set(CMAKE_CXX_STANDARD 14)

add_executable(LiDAR
        lidar.bin
        ParseLVX.cpp)
