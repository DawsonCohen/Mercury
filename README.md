# Simulation Instructions

## Optimization

### Build

1. edit Makefile's MAIN_SRC=main.cpp
1. export BUILD_LOCATION=1 for dialog sepretion for CUDA-12.0
1. make [-j12]

### Options
- TODO

## Visualization

### Build
sudo apt install libassimp-dev

1. edit CMakeList.txt set(PHYS main.cpp ...)
1. rm -r build
1. mkdir build
1. cd build
1. cp config.txt build
1. update OUT_DIR in the config (i.e. "../z_results")
1. cmake ..
1. make [-j12]

### Controls

- WASD: move camera
- Mouseclick: tilt camera
- TAB: switch solution

### Options
- VIDEO: enables visualization via OpenGL
- WRITE_VIDEO: writes video to file
- VISUALIZE: runs GUI for on-screen visualization