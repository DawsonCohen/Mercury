#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <vector>

namespace EvoDevo {

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

	enum ReplacementStrat {
		REPLACE_STANDARD = 0,
		PARETO = 1
	};

	enum NichingStrat {
		NICHE_NONE = 0,
		NICHE_HFC = 1,
		NICHE_ALPS = 2
	};

	enum RobotType {
		ROBOT_NN,
		ROBOT_VOXEL
	};

	enum EnvironmentType {
		ENVIRONMENT_LAND,
		ENVIRONMENT_WATER
	};

	enum CrossoverDistribution {
		CROSS_DIST_NONE = 0,
		CROSS_DIST_BINOMIAL = 1
	};

	enum CrossoverType {
		CROSS_INDIVIDUAL = 0,
		CROSS_CONTIGUOUS = 1
	};
	

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
			float time_step = 0.001f;
			EnvironmentType env_type = ENVIRONMENT_WATER;
		} simulator;

		struct Devo {
			float devo_time = 1.0f;
			uint devo_cycles = 0;
		} devo;

		struct SoftBody {
			unsigned int massCount = 1708;
		};

		struct NNRobot : public SoftBody {
			float crossover_neuron_count = .2;
			unsigned int mutation_weight_count = 10;
			unsigned int springs_per_mass = 25;
			std::vector<unsigned int> hidden_layer_sizes = {25,25};
			CrossoverDistribution crossover_distribution = CROSS_DIST_BINOMIAL;
			CrossoverType crossover_type = CROSS_CONTIGUOUS;
		} nnrobot;

		struct Hardware {
			std::vector<int> cuda_device_ids; // TODO
		} hardware;

		struct Optimizer {
			int pop_size = 512;
			int repeats = 1;
			int max_evals = 1e4;
			int niche_count = 4;
			int steps_to_combine = 100;
			int steps_to_exchange = 5000;
			int save_skip = 10;
			MutationStrat mutation = MUTATE;
			CrossoverStrat crossover = CROSS_SWAP;
			ReplacementStrat replacement = PARETO;
			NichingStrat niche = NICHE_NONE;
			float mutation_rate=0.6f;
			float crossover_rate=0.7f;
			float elitism=0.1f;
		} optimizer;

		struct Evaluator {
			int pop_size = 512;
			float base_time = 0.0;
			float eval_time = 10.0;
		} evaluator;
	};
	
}


#endif