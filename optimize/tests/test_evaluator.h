#include "Simulator.h"
#include "util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include "Evaluator.h"
#include "optimizer.h"
#include <Eigen/Core>

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

#define SIM_TIME 10.0f
#define ROBO_COUNT 10

int TestEvaluator() {
	OptimizerConfig config;
	config.evaluator.pop_size = ROBO_COUNT;
	std::vector<NNRobot> population(ROBO_COUNT);
	std::vector<float> og_fitness(ROBO_COUNT);

	for(auto& R : population) {
		R.Randomize();
	}

	Evaluator<NNRobot>::Initialize(config);
	
	std::vector<NNRobot> evalBuf(ROBO_COUNT);
    for(uint i = 0; i < population.size(); i++) {
        evalBuf[i] = population[i];
    }
    
    Evaluator<NNRobot>::BatchEvaluate(evalBuf);
	for(uint i = 0; i < population.size(); i++) {
		og_fitness[i] = evalBuf[i].fitness();
    }
	Evaluator<NNRobot>::BatchEvaluate(evalBuf);
	for(uint i = 0; i < population.size(); i++) {
		if(evalBuf[i].fitness() - og_fitness[i] > 1e-4) return 1;
    }

	return 0;
}