# EvoDevo - A Fast SoftBody Evolutionary Optimizer

## Requirements

- Linux 22.04
- gmp (sudo apt install libgmp-dev)

## Optimization

### Build

``````
git submodule add --init 
cd EvoDevo
make [-j12]
```````

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

## Requirements
libassimp-dev

### Build
``````
git submodule add --init --recursive
cd Visualizer
make [-j12]
./vis
```````

### Controls

- WASD: move camera
- Mouseclick: tilt camera
- TAB: switch solution
