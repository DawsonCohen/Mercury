cmake_minimum_required(VERSION 3.1)

set(CMAKE_CUDA_COMPILER "nvcc")
set(CMAKE_CXX_COMPILER "g++")

project(visualzie C CXX CUDA)

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

set(VISUALIZE main_visualize.cpp)

set(VISUALIZE
  ${VISUALIZE}
  ../common
  ../common/simulator/Simulator.cu

  ../common/evolvables/SoftBody.cpp
  ../common/evolvables/VoxelRobot.cpp
  ../common/evolvables/NNRobot.cpp
  ../common/evolvables/knn.cu

  ../common/util/util.cpp
  ../optimize/optimizer/util/optimizer_util.cpp
  util/visualizer_util.cpp
  application.cpp
)

set(RENDER
  renderer/src/Camera.cpp
  renderer/src/Shader.cpp
  renderer/src/Mesh.cpp
  renderer/src/Renderer.cpp
  renderer/src/InputManager.cpp
  renderer/src/VAO.cpp
  renderer/src/VBO.cpp
  renderer/src/EBO.cpp
  renderer/src/Model.cpp

  models/plane_model.cpp
)

# Create executable
if(DEFINED ENV{VIDEO} AND NOT BENCHMARK)
add_executable(visualizer ${VISUALIZE} ${RENDER})
else()
add_executable(visualizer ${VISUALIZE})
endif()

# Set CUDA architectures
set_target_properties(visualizer PROPERTIES CUDA_ARCHITECTURES "61;75")

# Set include directories
target_include_directories(visualizer PUBLIC
  ../common/lib
  ../common/simulator
  ../optimize/optimizer
  ../optimize/optimizer/util
  ../common/util
  ../common/evolvables
)

# Set include directories for local build
# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add subdirectory for renderer
add_subdirectory(renderer)

target_include_directories(visualizer PUBLIC
  renderer/src
  models
  util
)

# Link libraries
target_link_libraries(visualizer PUBLIC
  glad
  glfw
  glm
  stb
  ${OpenCV_LIBS}
)
