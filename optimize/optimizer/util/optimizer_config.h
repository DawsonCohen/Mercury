#ifndef __OPTIMIZER_CONFIG_H__
#define __OPTIMIZER_CONFIG_H__

#include <string>
#include <vector>

#include "config.h"

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

struct OptimizerConfig : public Config {
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

	OptimizerConfig() {}
	OptimizerConfig(const Config& config) : Config(config) {}
};

#endif