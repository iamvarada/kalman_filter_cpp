cmake_minimum_required(VERSION 3.4)
project(basci_kalman_filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(EIGEN_INC_DIR /usr/local/include/eigen3/)
include_directories(${EIGEN_INC_DIR})

set(SOURCE_FILES kalman_filter.cpp)
add_executable(basic_kalman_filter ${SOURCE_FILES})
