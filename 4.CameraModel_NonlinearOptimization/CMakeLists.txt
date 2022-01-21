cmake_minimum_required(VERSION 3.0)
project(disparity)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(OpenCV 3.1 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories("/usr/local/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(disparity disparity.cpp)
target_link_libraries(disparity ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBS})