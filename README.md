# Mercury
Mercury is a CUDA-based simulation library that forcuses on fast, cached simulation of soft-body robots on land in the water. See our paper [here](https://github.com/DawsonCohen/Mercury/blob/master/mercury_fall_2023.pdf) for full details

# Simulation Instructions
## Requirements:
1) Install c++/ [g++](https://linuxconfig.org/how-to-install-g-the-c-compiler-on-ubuntu-22-04-lts-jammy-jellyfish-linux)
- Check using gcc --version (Should be 11.4 on new machines)
2) Install cuda
- Download cuda using this [guide](https://developer.nvidia.com/cuda-downloads?target_os=Linux)
- Add to path using this instruction: export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}} (If using a different cuda version, change the number in the instruction to match)
- Check using nvcc --version (Should match the downloaded cuda version, 12.3 at time of writing)

3) Install the boost library
Use the following instruction: sudo apt-get install libboost-all-dev

4) Install Libgmp and copy it into common/lib
- sudo apt-get install libgmp3-dev
- Find where it's installed (dpkg -L libgmp3-dev)
- Move or copy all the files into common/lib (You need all the libgmp and libgmpxx files)

## Optimization
Note before building: You will need to customize the makefile to compile for your GPU architecture. On line 6 include the number for your GPU architecture.

### Build

1. cd optimize
1. make [-j12]
1. ./evodevo configs/config.default

### Config Options
**Optimizer Parameters**
- ROBOT_TYPE {NNRobot, VoxelRobot}
- REPEATS
- MAX_EVALS
- POP_SIZE
- NICHE_COUNT
- STEPS_TO_COMBINE
- STEPS_TO_EXCHANGE
- MUTATION {mutate, random}
- CROSSOVER {swap, dc, beam, none}
- NICHE {alps, hfc, none}
- MUTATION_RATE
- CROSSOVER_RATE
- ELITISM

**Evaluation Parameters**
- BASE_TIME
- EVAL_TIME

**Simulator Parameters**
- TRACK_STRESSES

**NN Robot**
- CROSSOVER_NEURONS
- MUTATION_WEIGHTS
- SPRINGS_PER_MASS
- HIDDEN_LAYER_SIZES

## Benchmarks
1. cd optimize
1. make [-j12] BENCHMARK=1
1. ./benchmark [options]

### Options
- voxel (default): voxel robots
- nn: nn robots
- build: nn robot build without simulation

## Visualization

### Build
sudo apt install libassimp-dev

1. cd visualize
1. cmake .
1. make [-j12]
1. ./visualizer configs/config.verify

### Controls

- WASD: move camera
- Mouseclick: tilt camera
- TAB: switch solution

### Config Options
- VERIFY	    opens robot solutions from folder
- WRITE_VIDEO:  writes video to file
- ZOO:          visualizes mulitple solutions at once
- BOUNCE        starts solutions above ground level
- STATIONARY    turns off gravity
