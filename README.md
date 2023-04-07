# Simulation Instructions

## Environment Setup
- On local machines (for rendering capablities):
  `export BUILD_LOCATION=local`
- On remote servers:
  - `export BUILD_LOCATION=remote`
  - Update set( CMAKE_CUDA_COMPILER /usr/local/cuda-XX.X/bin/nvcc )

## Initial Build

1. rm -r build
1. mkdir build
1. cd build
1. cmake ..
1. make

## Controls

- WASD
- Mouseclick
