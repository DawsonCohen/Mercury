cmake_minimum_required( VERSION 3.1 )
project( lib )

add_library(glad include/glad/glad.h src/glad.c)
add_library(stb include/stb/stb_image.h src/stb.cpp)

add_subdirectory( include/glm )
add_subdirectory( include/glfw )

target_include_directories(glad PUBLIC include)
target_include_directories(stb PUBLIC include)
