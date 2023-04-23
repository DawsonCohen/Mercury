cmake_minimum_required(VERSION 3.1)

set(CMAKE_CUDA_COMPILER "nvcc")
set(CMAKE_CXX_COMPILER "g++")

project(phys C CXX CUDA)

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
add_definitions("-DOPTIMIZE")
# add_definitions("-DOPTIMIZE -DVISUALIZE")
# add_definitions("-DVIDEO -DVISUALIZE")

# add_definitions("-DOPTIMIZE -DWRITE_VIDEO")
# add_definitions("-DVERIFY -DVISUALIZE")
# add_definitions("-DVERIFY -DWRITE_VIDEO")

#[[SIMULATOR OPTIONSSTRESS_COUNT - calculates each time a spring is the top/bottom stressed spring in a threadFULL_STRESS - calculates exact stress in each spring over the simulation lifetime]]#
# add_definitions("-DSTRESS_COUNT")
# add_definitions("-DSTRESS_COUNT -DFULL_STRESS -DWRITE_STRESS")

set(PHYS
	# batch_benchmark.cpp
	# benchmark.cpp
	main.cpp

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
if(DEFINED $ENV{VIDEO})
add_executable(phys ${PHYS} ${RENDER})
else()
add_executable(phys ${PHYS})
endif()

# Set CUDA architectures
set_target_properties(phys PROPERTIES CUDA_ARCHITECTURES "61;75")

# Set include directories
target_include_directories(phys PUBLIC
  lib
  simulator
  optimizer
  util
  evolvables
)

# Set include directories for local build
# Find OpenCV package
# find_package(OpenCV REQUIRED)

# Add subdirectory for renderer
if(DEFINED $ENV{VIDEO})
add_subdirectory(renderer)

target_include_directories(phys PUBLIC
  renderer/src
  models
)

# Link libraries
target_link_libraries(phys PUBLIC
  glad
  glfw
  glm
  stb
  ${OpenCV_LIBS}
)
endif()