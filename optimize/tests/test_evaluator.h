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

#define EPS 1e-4

std::vector<float> runEvaluator(std::vector<NNRobot> evalBuf) {
	std::vector<float> fitness(evalBuf.size());

    Evaluator<NNRobot>::BatchEvaluate(evalBuf);

	for(uint i = 0; i < evalBuf.size(); i++) {
		fitness[i] = evalBuf[i].fitness();
	}

	return fitness;
}

int TestEvaluator() {
	OptimizerConfig config;
	config.evaluator.pop_size = ROBO_COUNT;
	config.devo.devo_cycles = 1;
	std::vector<NNRobot> population(ROBO_COUNT);
	std::vector<float> og_fitness(ROBO_COUNT);
	std::vector<float> reset_fitness(ROBO_COUNT);
	std::vector<float> decode_fitness(ROBO_COUNT);

	for(auto& R : population) {
		R.Randomize();
	}

	Evaluator<NNRobot>::Initialize(config);
	
	std::vector<NNRobot> evalBuf(ROBO_COUNT);
    for(uint i = 0; i < population.size(); i++) {
        evalBuf[i] = population[i];
    }
    
	og_fitness = runEvaluator(evalBuf);
	reset_fitness = runEvaluator(evalBuf);
	
	int successflag = 0;
	for(uint i = 0; i < population.size(); i++) {
		printf("%f vs %f", og_fitness[i], reset_fitness[i]);
		if(abs(og_fitness[i] - reset_fitness[i]) > EPS) {
			printf(" FAILED");
			successflag += 1;
		}
		printf("\n");
    }

	

	printf("-----------------------------\n");
	printf("DECODED EVALUATION COMPARISON\n");
	std::vector<NNRobot> decode_evalBuf(ROBO_COUNT);
	for(uint i = 0; i < evalBuf.size(); i++) {
		std::string encoding = evalBuf[i].Encode();
		util::WriteCSV("test_NNBot_" + std::to_string(i) + ".csv","./z_results/",encoding);

		NNRobot Rcopy;
		std::string filepath = "./z_reslts/test_NNBot_" + std::to_string(i) + ".csv";
		Rcopy.Decode(filepath);
		decode_evalBuf[i] = Rcopy;	
	}
	decode_fitness = runEvaluator(decode_evalBuf);
	for(uint i = 0; i < ROBO_COUNT; i++) {
		printf("%f vs %f", og_fitness[i], decode_fitness[i]);
		if(abs(og_fitness[i] - decode_fitness[i]) > EPS) {
			printf(" FAILED");
			successflag += 1;
		}
		printf("\n");
    }

	return successflag;
}