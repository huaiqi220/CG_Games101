cmake_minimum_required (VERSION 2.8.11)
project (Transformation)



set(CMAKE_PREFIX_PATH /opt/homebrew/Cellar/eigen/3.4.0_1)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable (Transformation main.cpp)
