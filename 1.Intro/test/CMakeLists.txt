project(hello)

# generate a library called hellolib
add_library(hellolib hello.cpp)

add_executable(sayHello hello_world.cpp)
target_link_libraries(sayHello hellolib)
