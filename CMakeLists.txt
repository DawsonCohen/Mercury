cmake_minimum_required(VERSION 3.1)

set(CMAKE_CUDA_COMPILER "nvcc")
set(CMAKE_CXX_COMPILER "g++")

project(evodevo C CXX CUDA)

# Set language standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)


# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pg -W -Wall")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --ptxas-options=-v -ftz=true -prec-div=false -prec-sqrt=false --expt-relaxed-constexpr")

if("$ENV{BUILD_LOCATION}" STREQUAL "local")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20012")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20013")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20015")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20050")
endif()

## MAIN OPTIONS ##
if(DEFINED ENV{VIDEO})
  add_definitions("-DVIDEO")
endif()

if(BENCHMARK)
  message("Configuring for benchmarks")
  set(EVODEVO benchmark.cpp)
else()
  message("Configuring for optimization")
  set(EVODEVO main.cpp)
endif()

set(EVODEVO
  ${EVODEVO}
  simulator/Simulator.cu

  evolvables/SoftBody.cpp
  evolvables/VoxelRobot.cpp
  evolvables/NNRobot.cpp
  evolvables/knn.cu

  util/util.cpp
)

set(RENDER
  renderer/src/Camera.cpp
  renderer/src/Shader.cpp
  renderer/src/Mesh.cpp
  renderer/src/Renderer.cpp
  renderer/src/VAO.cpp
  renderer/src/VBO.cpp
  renderer/src/EBO.cpp
  renderer/src/Model.cpp

  models/plane_model.cpp
)

# Create executable
if(DEFINED ENV{VIDEO} AND NOT BENCHMARK)
add_executable(evodevo ${EVODEVO} ${RENDER})
else()
add_executable(evodevo ${EVODEVO})
endif()

# Set CUDA architectures
set_target_properties(evodevo PROPERTIES CUDA_ARCHITECTURES "61;75")

# Set include directories
target_include_directories(evodevo PUBLIC
  lib
  simulator
  optimizer
  util
  evolvables
)

if(DEFINED ENV{VIDEO} AND NOT BENCHMARK)
# Set include directories for local build
# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add subdirectory for renderer
add_subdirectory(renderer)

target_include_directories(evodevo PUBLIC
  renderer/src
  models
)

# Link libraries
target_link_libraries(evodevo PUBLIC
  glad
  glfw
  glm
  stb
  ${OpenCV_LIBS}
)
endif()
