#include "Simulator.h"
#include "VoxelRobot.h"
#include "NNRobot.h"
#include "util.h"

#include <thread>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

// #define MAX_TIME 5
// #define POP_SIZE 1

// TODO: Use config file

#define MAX_TIME 5
#define MAX_SPRINGS 1e11
#define INIT_POP_SIZE 1

void VoxelBenchmark();
void NNBenchmark();
void NNBuildBenchmark();
Simulator sim;

using namespace std::chrono_literals;

std::string out_dir;

int main(int argc, char** argv)
{
	// sim.Initialize(solution.masses.size(),solution.springs.size(), POP_SIZE);
	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);

	// Get current date and time
	auto now = std::chrono::system_clock::now();
	time_t current_time = std::chrono::system_clock::to_time_t(now);
	// Convert current time to a string with format YYYY-MM-DD-HHMMSS
	char time_str[20];
	strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H%M%S", localtime(&current_time));

	util::MakeDirectory("../z_results");
	util::MakeDirectory("../z_results/benchmarks");

	out_dir = std::string("../z_results/benchmarks/") + std::string(time_str);
	util::MakeDirectory(out_dir);

	if(argc > 1) {
		if(std::string(argv[1]) == std::string("nn"))
			NNBenchmark();
		else if(std::string(argv[1]) == std::string("build"))
			NNBuildBenchmark();
		else
			VoxelBenchmark();
	} else {
		VoxelBenchmark();
	}
	
	return 0;
}

void VoxelBenchmark() {
	printf("BENCHMARKING VOXELS\n");
	VoxelRobot R;

	uint pop_size = INIT_POP_SIZE;
	sim.Initialize(R,pop_size);

	ulong num_springs = R.getSprings().size() * (sim.getMaxTime() / sim.getDeltaT());

	FILE* pFile = fopen((out_dir + "/voxel_benchmark.csv").c_str(),"w");

	float execute_time;

	fprintf(pFile,"springs simulated, springs per iteration, execute time\n");

	while(num_springs < MAX_SPRINGS) {
		std::vector<Element> robots;
		for(uint i = 0; i < pop_size; i++) {
			robots.push_back({R.getMasses(), R.getSprings()});
		}

		// printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		
		sim.Initialize(R,pop_size);
		sim.setMaxTime(MAX_TIME);

		printf("STARTED\n");
		auto start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(robots);
		auto end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = R.getSprings().size() * pop_size * (sim.getMaxTime() / sim.getDeltaT());
		float springs_per_sec = num_springs / execute_time;

		fprintf(pFile,"%lu,%lu,%f\n", num_springs,R.getSprings().size()*pop_size,execute_time);
		printf("%u ROBOTS (%.2e SPRINGS) IN %f SECONDS\n", pop_size, (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
	fclose(pFile);
}

void NNBuildBenchmark() {
	printf("BENCHMARKING\n");

	NNRobot R;

	uint pop_size = INIT_POP_SIZE;
	sim.Initialize(R, pop_size);

	ulong num_springs = R.getSprings().size() * (sim.getMaxTime() / sim.getDeltaT());
	FILE* pFile = fopen((out_dir + "/nnbuild_benchamrk.csv").c_str(),"w");

	float execute_time;

	// fprintf(pFile,"springs simulated, springs per iteration, execute time\n");

	while(num_springs <= MAX_SPRINGS) {
		std::vector<NNRobot> robots;
		std::vector<Element> robot_elements;

		auto start = std::chrono::high_resolution_clock::now();
		for(uint i = 0; i < pop_size; i++) {
			NNRobot R;
			robots.push_back(R);
		}
		auto end = std::chrono::high_resolution_clock::now();

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = R.getSprings().size() * pop_size * (sim.getMaxTime() / sim.getDeltaT());
		printf("BUILT %u ROBOTS IN %f SECONDS\n", pop_size, execute_time);


		pop_size *= 2;
	}
	fclose(pFile);
}

void NNBenchmark() {
	printf("BENCHMARKING\n");

	NNRobot R;
	R.Build();

	uint pop_size = INIT_POP_SIZE;
	sim.Initialize(R.maxMasses, R.maxSprings, pop_size);

	ulong num_springs = R.getSprings().size() * (sim.getMaxTime() / sim.getDeltaT());
	FILE* pFile = fopen((out_dir + "/nn_benchamrk.csv").c_str(),"w");

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
		
		sim.Initialize(R,pop_size);
		sim.setMaxTime(MAX_TIME);

		printf("STARTED\n");
		start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(robot_elements);
		end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = R.getSprings().size() * pop_size * (sim.getMaxTime() / sim.getDeltaT());
		float springs_per_sec = num_springs / execute_time;

		fprintf(pFile,"%lu,%lu,%f\n", num_springs, R.getSprings().size()*pop_size,execute_time);
		printf("%u ROBOTS (%.2e SPRINGS) IN %f SECONDS\n", pop_size, (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
	fclose(pFile);
}