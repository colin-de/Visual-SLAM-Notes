cmake_minimum_required(VERSION 3.1)
project(hello_slam)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build Type : ${CMAKE_BUILD_TYPE}")

add_executable(sayHello src/useHello.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

# libhello.0
add_library(hello SHARED ${PROJECT_SOURCE_DIR}/src/hello.cpp)

target_link_libraries(sayHello PUBLIC hello)


