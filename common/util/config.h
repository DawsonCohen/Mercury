#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <vector>

enum RobotType {
	ROBOT_NN,
	ROBOT_VOXEL
};

enum MutationStrat {
	MUTATE_RANDOM = 0,
	MUTATE = 1
};

enum MutationRateStrat {
	MUTATION_RATE_CONST = 0,
	MUTATION_RATE_ANNEAL = 1
};

enum CrossoverStrat {
	CROSS_NONE = -1,
	CROSS_BEAM = 0,
	CROSS_SWAP = 1,
	CROSS_DC = 2
};

enum NichingStrat {
	NICHE_NONE = 0,
	NICHE_HFC = 1,
	NICHE_ALPS = 2
};

struct Config {
	RobotType robot_type = ROBOT_NN;
	struct IO {
		std::string in_dir = "";
		std::string out_dir = "./z_results";
	} io;
	struct Objectives {
		bool optimize = true;
		bool visualize = false;
		bool verify = false;
		bool zoo = false;
		bool bounce = false;
		bool stationary = false;
		bool movie = false;
	} objectives;
	struct Optimizer {
		int pop_size = 512;
		int repeats = 1;
		int max_evals = 1e4;
		int niche_count = 4;
		int steps_to_combine = 100;
		int steps_to_exchange = 5000;
		MutationStrat mutation = MUTATE;
		CrossoverStrat crossover = CROSS_SWAP;
		NichingStrat niche = NICHE_NONE;
		float mutation_rate=0.6f;
		float crossover_rate=0.7f;
		float elitism=0.1f;
	} optimizer;
	struct Evaluator {
		int pop_size = 512;
		float base_time = 3.0;
		float eval_time = 10.0;
		float devo_time = 1.0;
		uint replace_amount = 100;
		uint devo_cycles = 100;
	} evaluator;
	struct Simulator {
		bool track_stresses = false;
		bool devo = false;
		bool visual = false;
		uint replacedSpringsPerElement = 32;
	} simulator;

	struct Renderer {
		float max_time = 10.0;
		float fps = 30;
		int	  width = 1600;
		int   height = 900;
	} renderer;

	struct NNRobot {
		int crossover_neuron_count = 5;
		int mutation_weight_count = 10;
		int springs_per_mass = 25;
		std::vector<int> hidden_layer_sizes = {25,25};
	} nnrobot;

	struct Hardware {
		std::vector<int> cuda_device_ids; // TODO
	} hardware;
};

#endif