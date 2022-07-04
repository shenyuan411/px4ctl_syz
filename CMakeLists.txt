cmake_minimum_required(VERSION 2.8)
project(tag)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3 -fopenmp -pthread")

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


enable_testing()

################### source and driver ######################

add_executable(tag
               tag_det.cpp)

target_link_libraries(tag
                      ${OpenCV_LIBS})
