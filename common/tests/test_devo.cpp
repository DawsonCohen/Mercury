#include "tests.h"
#include "NNRobot.h"

#define SIM_TIME 0.0f
#define DEVO_TIME 0.01f
#define ROBO_COUNT 1024
#define DEVO_CYCLES 10

int TestDevo() {
	Config config;
	Simulator sim;

	std::vector<SoftBody> robots;

	NNRobot R;
	R.Randomize();
	for(uint i = 0; i < ROBO_COUNT; i++) {
		robots.push_back(R);
	}

	config.simulator.time_step = 5e-3;
	sim.Initialize(config.simulator);

	std::vector<float> og_fitness = runSimulator(sim, robots, SIM_TIME, DEVO_CYCLES, DEVO_TIME);
	for(uint i = 0; i < 1000000; i++) {
		sim.Devo();
	}

	int successFlag = 0; // default passed

	return successFlag;
}