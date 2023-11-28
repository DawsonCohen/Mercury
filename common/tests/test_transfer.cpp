#include "common_tests.h"
#include "NNRobot.h"

#define ROBO_COUNT 512
#define CYCLES 20

void cycleSimulator(Simulator& sim, std::vector<SoftBody> robots) {
	sim.Reset();
	
	std::vector<ElementTracker> trackers;
	std::vector<Element> elements;
	std::vector<Element> results;

	static int run = 0;
	std::string tracename = "sim_trace_" + std::to_string(run) + ".csv";

    for(auto& R : robots) {
        elements.push_back(R);
    }

	trackers = sim.SetElements(elements);

	sim.Collect(trackers);
}

int TestTransfer() {
	Config config;
	Simulator sim;

	std::vector<SoftBody> robots;

	NNRobot R;
	R.Randomize();
	R.Build();
	for(size_t i = 0; i < ROBO_COUNT; i++) {
		robots.push_back(R);
	}

    for(size_t i = 0; i < CYCLES; i++) {
        cycleSimulator(sim, robots);
    }

	int successFlag = 0; // default passed

	return successFlag;
}