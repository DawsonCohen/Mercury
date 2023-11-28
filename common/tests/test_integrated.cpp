#include "Simulator.h"
#include "util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>
#include "SoftBody.h"

#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include "common_tests.h"

#define SIM_TIME 5.0f
#define ROBO_COUNT 10

std::vector<float> runIntegrated(Simulator& sim, std::vector<SoftBody> robots, bool trace = false) {
	sim.Reset();
	std::vector<float> fitnesses(robots.size());
	
	std::vector<ElementTracker> trackers;
	std::vector<Element> elements;
	std::vector<Element> results;

	static int run = 0;
	std::string tracename = "sim_trace_" + std::to_string(run) + ".csv";

	uint devoCycles = 1;
	float devoTime = 1.0f;

	std::vector<bool> robotWasAllocated(robots.size());
    uint i = 0;
    for(auto& R : robots) {
        R.Reset();
		robotWasAllocated[i] = R.isValid();
        if(R.isValid()) elements.push_back(R);

        i++;
    }

	trackers = sim.SetElements(elements);

	for(uint i = 0; i < devoCycles; i++) {
        sim.Simulate(devoTime, true, trace, tracename);

        sim.Devo();
    }

	sim.Simulate(SIM_TIME,false,trace, tracename);

	results = sim.Collect(trackers);
	uint skip_count = 0;
	for(uint i = 0; i < robots.size(); i++) {
        if(robotWasAllocated[i]) {
            robots[i].Update(results[i - skip_count]);
        } else {
            skip_count++;
        }
		fitnesses[i] = robots[i].fitness();
    }

	if(trace) run += 1;

	return fitnesses;
}

std::string IntegratedEncode(SoftBody S) {
    std::stringstream ss;
    ss << "type=NNRobot\n";
    ss << "masses=";
    for(unsigned int i = 0; i < S.masses.size(); i++) {
        ss << S.masses[i];
        ss << (i < (S.masses.size()- 1) ? ";" : "\n");
    }
    ss << "springs=";
    for(unsigned int i = 0; i < S.springs.size(); i++) {
        ss << S.springs[i];
        ss << (i < S.springs.size()- 1 ? ";" : "\n");
    }
    return ss.str();
}

int TestIntegrated() {
	Config config;
	Simulator sim;

	std::vector<SoftBody> robots;
	std::vector<SoftBody> robots2;

	for(uint i = 0; i < ROBO_COUNT; i++) {
		NNRobot R;
		R.Randomize();
		R.Build();
		robots.push_back(R);
	}

	config.simulator.time_step = 1e-3;
	sim.Initialize(config.simulator);

	std::vector<float> og_fitness = runIntegrated(sim, robots);
	int successflag = 0;

	for(uint i = 0; i < ROBO_COUNT; i++){
		SoftBody R = robots[i];
		std::string encoding = IntegratedEncode(robots[i]);

	    util::WriteCSV("test_NNBot.csv","./z_results/",encoding);

	    NNRobot Rdecoded;
	    Rdecoded.Decode("./z_results/test_NNBot.csv");
	    robots2.push_back(Rdecoded);
	    for(size_t i = 0; i < Rdecoded.masses.size(); i++) {
			if((R.masses[i].pos - Rdecoded.masses[i].pos).norm() > 0) {
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
	printf("successflag: %u\n", successflag);

	std::vector<float> reset_fitness = runIntegrated(sim, robots2);

	int successFlag = 0; // default passed
	unsigned int maxDiffId = 0;
	float maxDiff = 0;
	for(uint i = 0; i < robots.size(); i++) {
		printf("OG Fitness: %f, Reset Fitness: %f", og_fitness[i], reset_fitness[i]);
		if(abs(reset_fitness[i] -  og_fitness[i]) > 1e-4) {
			successFlag += 1; // failure
			printf(" FAILED");
		}
		if(abs(reset_fitness[i] -  og_fitness[i]) > maxDiff) {
			maxDiffId = i;
			maxDiff = abs(robots2[i].fitness() -  og_fitness[i]);
		}
		printf("\n");
	}
	printf("Robot %u maxDiff %f - OG Fitness: %f, Reset Fitness: %f\n",maxDiffId, maxDiff,og_fitness[maxDiffId],reset_fitness[maxDiffId]);

	robots[maxDiffId].Reset();
	std::vector<SoftBody> maxDiffRobot = {robots[maxDiffId]};
	og_fitness = runIntegrated(sim, maxDiffRobot);
	maxDiffRobot[0].Reset();
	reset_fitness = runIntegrated(sim, maxDiffRobot);
	printf("Second run comparison for max diff robot: %f vs %f\n",og_fitness[0], reset_fitness[0]);

	return successFlag;
}