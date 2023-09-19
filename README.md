# Simulation Instructions

## Optimization

### Build

1. export BUILD_LOCATION=local `#for dialog sepretion for CUDA-12.0`
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


## Visualization

### Build
sudo apt install libassimp-dev

1. export VIDEO=1 `#to include video library`
1. mkdir build
1. cp -r configs build
1. cd build
1. cmake ..
1. make [-j12]
1. ./evodevo configs/config.default

### Controls

- WASD: move camera
- Mouseclick: tilt camera
- TAB: switch solution

### Config Options
- VERIFY	    opens robot solutions from folder
- WRITE_VIDEO:  writes video to file
- VISUALIZE:    runs GUI for on-screen visualization
- ZOO:          visualizes mulitple solutions at once
- BOUNCE        starts solutions above ground level
- STATIONARY    turns off gravity

## Benchmark
1. mkdir build
1. cd build
1. cmake -DBENCHMARK=ON ..
1. make [j12]
1. ./evodevo [options]

### Options
- voxel (default): voxel robots
- nn: nn robots
- build: nn robot build without simulation