# Visualizer executable
cmake_minimum_required(VERSION 3.11)

set(CMAKE_CUDA_COMPILER "nvcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pg -W -Wall")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --ptxas-options=-v -ftz=true -prec-div=false -prec-sqrt=false --expt-relaxed-constexpr")

file(GLOB VISUALIZER_SRC
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Core/*.cpp
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Evolvables/*.cpp
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Evolvables/*.cu
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Simulator/*.cpp
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Simulator/*.cu
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Util/*.cpp
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/Util/*.cpp
    ${CMAKE_SOURCE_DIR}/src/Models/*.cpp
)

file(GLOB VISUALIZER_HEAD
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Core/*.h
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Evolvabels/*.h
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Optimizer/*.h
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Simulator/*.h
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Simulator/*.cuh
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/EvoDevo/Util/*.h
    ${CMAKE_SOURCE_DIR}/src/*.h
    ${CMAKE_SOURCE_DIR}/src/Util/*.h
    ${CMAKE_SOURCE_DIR}/src/Models/*.h
)

project(Visualizer C CXX CUDA)

add_subdirectory(./vendor/elastic build)

add_executable(vis
    # Add all source files
    ${VISUALIZER_SRC}
    ${VISUALIZER_HEAD}
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src/evpch.cpp
)

set_target_properties(vis PROPERTIES CUDA_ARCHITECTURES "61;75")

include_directories(vis PRIVATE 
    ${CMAKE_SOURCE_DIR}/../EvoDevo/src
    ${CMAKE_SOURCE_DIR}/../EvoDevo/vendor
    ${CMAKE_SOURCE_DIR}/../EvoDevo/vendor/eigen
    ${CMAKE_SOURCE_DIR}/../EvoDevo/vendor/CGAL-5.6/include
    ${CMAKE_SOURCE_DIR}/../Visualizer/vendor/elastic/Elastic/vendor/spdlog/include
    ${CMAKE_SOURCE_DIR}/../Visualizer/vendor/elastic/Elastic/vendor/
    ${CMAKE_SOURCE_DIR}/../Visualizer/vendor/elastic/Elastic/vendor/glm
    ${CMAKE_SOURCE_DIR}/../Visualizer/vendor/elastic/Elastic/src
)

target_include_directories(vis PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(vis PUBLIC gmp pthread Elastic)

# Compiler options for different configurations
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DEL_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DEL_RELEASE -O3")
set(CMAKE_CXX_FLAGS_DIST "${CMAKE_CXX_FLAGS_DIST} -DEL_DIST -O3")