cmake_minimum_required(VERSION 2.8.3)
project(TagAidedLocalization)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge
        message_filters
        image_transport
)

find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Eigen3)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# ################## source and driver ######################
add_executable(tag
        src/tag_det.cpp)

target_link_libraries(tag
        ${OpenCV_LIBS})

add_executable(tag_aided_loc src/tag_aided_loc_node)
target_link_libraries(tag_aided_loc ${catkin_LIIBRARIES} ${OpenCV_LIBS})