#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <vector>

#include "structs.h"

struct Config {
	RobotType robot_type = ROBOT_NN;
	struct IO {
		std::string in_dir = "";
		std::string base_dir = "./z_results";
		std::string out_dir = "./z_results";
	} io;
	struct Simulator {
		bool visual = false;
		uint replaced_springs_per_element = 128;
		float time_step = 0.005f;
		EnvironmentType env_type = ENVIRONMENT_WATER;
	} simulator;

	struct Devo {
		float devo_time = 1.0f;
		uint devo_cycles = 0;
	} devo;

	struct SoftBody {
		unsigned int massCount = 1728;
	};

	struct NNRobot : public SoftBody {
		unsigned int crossover_neuron_count = 5;
		unsigned int mutation_weight_count = 10;
		unsigned int springs_per_mass = 25;
		std::vector<unsigned int> hidden_layer_sizes = {25,25};
		CrossoverDistribution crossover_distribution = CROSS_DIST_BINOMIAL;
		CrossoverType crossover_type = CROSS_CONTIGUOUS;
	} nnrobot;

	struct Hardware {
		std::vector<int> cuda_device_ids; // TODO
	} hardware;
};

#endif