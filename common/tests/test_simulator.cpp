#include "Simulator.h"
#include "util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>

#include <regex>
#include <thread>
#include <chrono>
// #include <iostream>
#include <string>
#include <sys/stat.h>

#include "common_tests.h"

#define SIM_TIME 5.0f
#define ROBO_COUNT 10

int TestSimulator() {
	Config config;
	Simulator sim;

	std::vector<SoftBody> robots;

	for(uint i = 0; i < ROBO_COUNT; i++) {
		VoxelRobot R;
		R.Randomize();
		R.Build();
		robots.push_back(R);
	}

	config.simulator.time_step = 1e-3;
	sim.Initialize(config.simulator);

	std::vector<float> og_fitness = runSimulator(sim, robots, SIM_TIME);
	std::vector<float> reset_fitness = runSimulator(sim, robots, SIM_TIME);

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
			maxDiff = abs(robots[i].fitness() -  og_fitness[i]);
		}
		printf("\n");
	}
	printf("Robot %u maxDiff %f - OG Fitness: %f, Reset Fitness: %f\n",maxDiffId, maxDiff,og_fitness[maxDiffId],reset_fitness[maxDiffId]);

	robots[maxDiffId].Reset();
	std::vector<SoftBody> maxDiffRobot = {robots[maxDiffId]};
	og_fitness = runSimulator(sim, maxDiffRobot, true);
	maxDiffRobot[0].Reset();
	reset_fitness = runSimulator(sim, maxDiffRobot, true);
	printf("Second run comparison for max diff robot: %f vs %f\n",og_fitness[0], reset_fitness[0]);

	return successFlag;
}

int TestMatEncoding() {
    VoxelRobot R;
    Material bone = materials::bone;
    Material add = materials::adductor_muscle0;
    Material air = materials::air;
    Material air_decoded = materials::decode(air.encoding);
    Material bone_decoded = materials::decode(bone.encoding);
    Material add_decoded = materials::decode(add.encoding);
    Material ab_decoded = materials::decode(air.encoding | bone.encoding);
    
    if(air_decoded != materials::air) return 1;
    if(bone_decoded != materials::bone) return 2;
    if(add_decoded != materials::adductor_muscle0) return 2;
    if(ab_decoded != materials::air) return 3;

    return 0;
}
