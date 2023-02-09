#include "Simulator.h"
// #include "VoxelRobot.h"
#include "robot.h"

#include <thread>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

// #define MAX_TIME 5
// #define POP_SIZE 1

#define MAX_TIME 5
#define MAX_SPRINGS 2e11
#define INIT_POP_SIZE 1

void Benchmark(Robot& R);
Simulator sim;

using namespace std::chrono_literals;

int main()
{
	printf("BENCHMARKING\n");
	
	Robot solution = Robot();

	// sim.Initialize(solution.masses.size(),solution.springs.size(), POP_SIZE);
	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
	
	printf("%lu masses, %lu springs\n",solution.masses.size(), solution.springs.size());
	
	Benchmark(solution);
	
	return 0;
}

void Benchmark(Robot& R) {
	// printf("SIMULATING\n");

	uint pop_size = INIT_POP_SIZE;
	sim.Initialize(R,pop_size);

	ulong num_springs = R.getSprings().size() * (sim.getMaxTime() / sim.getStepPeriod());
	FILE* pFile = fopen("../z_results/speed_test.csv","w");

	// sim = Simulator(R,pop_size);

	float execute_time;

	while(num_springs < MAX_SPRINGS) {
		std::vector<Element> robots;
		for(uint i = 0; i < pop_size; i++) {
			robots.push_back({R.getMasses(), R.getSprings()});
		}

		printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		
		sim.Initialize(R,pop_size);
		sim.setMaxTime(MAX_TIME);

		printf("STARTED\n");
		auto start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(robots);
		auto end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = R.getSprings().size() * pop_size * (sim.getMaxTime() / sim.getStepPeriod());
		float springs_per_sec = num_springs / execute_time;

		pop_size *= 2;

		fprintf(pFile,"%lu, %f\n", num_springs,execute_time);
		printf("%lu SPRINGS IN %f SECONDS\n", num_springs, execute_time);
		printf("%lu SPRINGS PER SECOND\n", (ulong) springs_per_sec);
		printf("--------------------------\n");
	}
	fclose(pFile);
}
