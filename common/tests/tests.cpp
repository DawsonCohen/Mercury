#include "tests.h"
#include <array>

std::vector<float> runSimulator(Simulator& sim, std::vector<SoftBody> robots, float simTime, bool trace) {
    std::vector<float> fitnesses = runSimulator(sim, robots, simTime, 0, 0.0f, trace);
	return fitnesses;
}
    
std::vector<float> runSimulator(Simulator& sim, std::vector<SoftBody> robots, float simTime, uint devoCycles, float devoTime, bool trace) {
	sim.Reset();
	std::vector<float> fitnesses(robots.size());
	
	std::vector<ElementTracker> trackers;
	std::vector<Element> elements;
	std::vector<Element> results;

	static int run = 0;
	std::string tracename = "sim_trace_" + std::to_string(run) + ".csv";

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

	sim.Simulate(simTime,false,trace, tracename);

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