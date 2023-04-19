#include "Simulator.h"
#include "NNRobot.h"

#include <thread>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

// #define MAX_TIME 5
// #define POP_SIZE 1

#define MAX_TIME 5
#define INIT_POP_SIZE 8
#define END_POP_SIZE 8

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
	FILE* pFile = fopen("./z_results/build_test.csv","w");

	float execute_time;

	// fprintf(pFile,"springs simulated, springs per iteration, execute time\n");

	while(pop_size <= END_POP_SIZE) {
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

		pop_size *= 2;
	}
	fclose(pFile);
}
