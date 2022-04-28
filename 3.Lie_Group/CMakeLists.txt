# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")
cmake_minimum_required(VERSION 3.0)
project(use_pangolin)

# Add package functions
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/local/include/eigen3")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory Sophus::Sophus ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})