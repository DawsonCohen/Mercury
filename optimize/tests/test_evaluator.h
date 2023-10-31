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

#define EPS 0

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
	
	for(uint i = 0; i < ROBO_COUNT; i++) {
		std::stringstream out;
		out << std::setprecision(9) << og_fitness[i] << " vs " << reset_fitness[i];
		printf("%s", out.str().c_str());
		float diff = og_fitness[i] - reset_fitness[i];
		if(diff > EPS || diff < -EPS) {
			std::stringstream fail;
			fail << std::setprecision(9) << "FAILED with diff: " << diff;
			printf("%s", fail.str().c_str());
			successflag += 1;
		}
		printf("\n");
    }

	

	printf("-----------------------------\n");
	printf("DECODED EVALUATION COMPARISON\n");
	std::vector<NNRobot> decode_evalBuf(ROBO_COUNT);
	for(uint i = 0; i < evalBuf.size(); i++) {
		NNRobot R = evalBuf[i];
		NNRobot Rdecoded;
		std::string encoding = evalBuf[i].Encode();
		util::WriteCSV("test_NNBot_" + std::to_string(i) + ".csv","./z_results/",encoding);

		std::string filepath = "./z_results/test_NNBot_" + std::to_string(i) + ".csv";
		Rdecoded.Decode(filepath);
		decode_evalBuf[i] = Rdecoded;

		for(size_t i = 0; i < Rdecoded.masses.size(); i++) {
			if((R.masses[i].pos - Rdecoded.masses[i].pos).norm() > EPS) {
				Eigen::Vector3f pos1 = R.masses[i].pos;
				Eigen::Vector3f pos2 = Rdecoded.masses[i].pos;
				printf("(%f,%f,%f) vs (%f,%f,%f)\n",pos1[0],pos1[1],pos1[2],pos2[0],pos2[1],pos2[2]);
				successflag += 1;
			}
			if(R.masses[i].material != Rdecoded.masses[i].material) {
				printf("%lu: %u vs %u\n",i, R.masses[i].material.encoding, Rdecoded.masses[i].material.encoding);
				successflag += 1;
			}
		}

	}
	decode_fitness = runEvaluator(decode_evalBuf);
	for(uint i = 0; i < ROBO_COUNT; i++) {
		std::stringstream out;
		out << std::setprecision(9) << og_fitness[i] << " vs " << reset_fitness[i];
		printf("%s", out.str().c_str());
		float diff = og_fitness[i] - reset_fitness[i];
		if(diff > EPS || diff < -EPS) {
			std::stringstream fail;
			fail << std::setprecision(9) << "FAILED with diff: " << diff;
			printf("%s", fail.str().c_str());
			successflag += 1;
		}
		printf("\n");
    }

	return successflag;
}