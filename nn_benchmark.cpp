#include "Simulator.h"
#include "NNRobot.h"

#include <thread>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

// #define MAX_TIME 5
// #define POP_SIZE 1

#define MAX_TIME 5
#define MAX_SPRINGS 1e11
#define INIT_POP_SIZE 1

void Benchmark();
Simulator sim;

using namespace std::chrono_literals;

int main()
{
	// sim.Initialize(solution.masses.size(),solution.springs.size(), POP_SIZE);
	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
	
	Benchmark();
	
	return 0;
}

void Benchmark() {
	printf("BENCHMARKING\n");

	NNRobot protoRobot;
	protoRobot.Build();

	uint pop_size = INIT_POP_SIZE;
	sim.Initialize(protoRobot.maxMasses, protoRobot.maxSprings, pop_size);

	ulong num_springs = protoRobot.getSprings().size() * (sim.getMaxTime() / sim.getStepPeriod());
	FILE* pFile = fopen("./z_results/speed_test.csv","w");

	float execute_time;

	fprintf(pFile,"springs simulated, springs per iteration, execute time\n");

	while(num_springs < MAX_SPRINGS) {
		std::vector<NNRobot> robots;
		std::vector<Element> robot_elements;

		auto start = std::chrono::high_resolution_clock::now();
		for(uint i = 0; i < pop_size; i++) {
			NNRobot R;
			R.Build();
			robots.push_back(R);
		}
		auto end = std::chrono::high_resolution_clock::now();

		execute_time = std::chrono::duration<float>(end - start).count();
		printf("BUILT %u ROBOTS IN %f SECONDS\n", pop_size, execute_time);

		// NNRobot::BatchBuild(robots);

		for(auto& R : robots) {
			robot_elements.push_back({R.getMasses(), R.getSprings()});
		}

		
		// printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		
		sim.Initialize(protoRobot,pop_size);
		sim.setMaxTime(MAX_TIME);

		printf("STARTED\n");
		start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(robot_elements);
		end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = protoRobot.getSprings().size() * pop_size * (sim.getMaxTime() / sim.getStepPeriod());
		float springs_per_sec = num_springs / execute_time;

		fprintf(pFile,"%lu,%lu,%f\n", num_springs, protoRobot.getSprings().size()*pop_size,execute_time);
		printf("%u ROBOTS (%.2e SPRINGS) IN %f SECONDS\n", pop_size, (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
	fclose(pFile);
}
