#include "Simulator.h"
#include "util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

#define SIM_TIME 5.0f

int TestSimulator(std::vector<SoftBody>& robots) {
	Config config;
	Simulator sim;

	sim.Initialize(robots[0], robots.size(), config.simulator);

	float time_step = 1 / config.simulator.time_step;
	uint devo_cycles = config.devo.devo_cycles;
	float devo_time = config.devo.devo_time;
	float timeToDevo = devo_time;

	// Main while loop
	uint i = 0;
	
	std::vector<ElementTracker> trackers;
	std::vector<Element> elements;
	std::vector<Element> results;

	std::vector<float> og_fitness;

	for(auto& R : robots) {
		if(R.isValid()) elements.push_back(R);
	}
	
	sim.Reset();
	trackers = sim.SetElements(elements);
	sim.Simulate(SIM_TIME);

	results = sim.Collect(trackers);
	for(uint i = 0; i < robots.size(); i++) {
		robots[i].Update(results[i]);
		og_fitness.push_back(robots[i].fitness());
	}

	elements.clear();
	for(auto& R : robots) {
		R.Reset();
		if(R.isValid()) elements.push_back(R);
	}
	sim.Reset();

	trackers = sim.SetElements(elements);
	sim.Simulate(SIM_TIME);
	results = sim.Collect(trackers);

	int successFlag = 0; // default passed
	for(uint i = 0; i < robots.size(); i++) {
		robots[i].Update(results[i]);
		printf("OG Fitness: %f, Reset Fitness: %f", og_fitness[i], robots[i].fitness());
		if((robots[i].fitness() -  og_fitness[i]) > 1e-5) {
			successFlag = 1; // failure
			printf(" FAILED");
		}
		printf("\n");
	}
	return successFlag;
}