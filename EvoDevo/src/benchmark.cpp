#include "EvoDevo.h"

#include <thread>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

#define MAX_TIME 0.1
#define MAX_SPRINGS 1e10
#define MAX_ROBOTS 512
#define INIT_POP_SIZE 1

using namespace EvoDevo;

void VoxelBenchmark();
void StressBenchmark();
void DevoBenchmark();
void NNBenchmark();
void NNBuildBenchmark();
Simulator sim;

using namespace std::chrono_literals;

std::string out_dir;
FileWriter fw;

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

	out_dir = std::string("./z_results/") + std::string(time_str);

	if(argc > 1) {
		if(std::string(argv[1]) == std::string("voxel"))
			VoxelBenchmark();
		else if(std::string(argv[1]) == std::string("nn"))
			NNBenchmark();
		else if(std::string(argv[1]) == std::string("build"))
			NNBuildBenchmark();
		else if(std::string(argv[1]) == std::string("stress"))
			StressBenchmark();
		else if(std::string(argv[1]) == std::string("devo"))
			DevoBenchmark();
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

	ulong num_springs = R.getSprings().size() * (MAX_TIME / sim.getDeltaT());

	fw.Open(out_dir + "/voxel_benchmark.csv");
	fw << "springs simulated, execute time, springs per second\n";

	float execute_time;


	while(num_springs < MAX_SPRINGS) {
		std::vector<Element> robots;
		for(uint i = 0; i < pop_size; i++) {
			robots.push_back(R);
		}

		// printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		
		sim.SetElements(robots);

		printf("STARTED\n");
		auto start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(MAX_TIME);
		auto end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = 0;
		for(auto& R : robots) {
			num_springs += R.springs.size();
		}
		num_springs = num_springs * (MAX_TIME / sim.getDeltaT());
		float springs_per_sec = num_springs / execute_time;

		fw << num_springs << "," << execute_time << "," << springs_per_sec << "\n";
		
		printf("%u ROBOTS (%.2e SPRINGS) IN %f SECONDS\n", pop_size, (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
}

void StressBenchmark() {
	printf("BENCHMARKING VOXEL STRESSES\n");
	VoxelRobot R;

	uint pop_size = INIT_POP_SIZE;

	ulong num_springs = R.getSprings().size() * (MAX_TIME / sim.getDeltaT());

	fw.Open(out_dir + "/stress_benchmark.csv");
	fw << "springs simulated, execute time, springs per second\n";

	float execute_time;


	while(num_springs < MAX_SPRINGS) {
		std::vector<Element> robots;
		for(uint i = 0; i < pop_size; i++) {
			robots.push_back(R);
		}

		// printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		
		sim.SetElements(robots);

		printf("STARTED\n");
		auto start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(MAX_TIME, true);
		auto end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = 0;
		for(auto& R : robots) {
			num_springs += R.springs.size();
		}
		num_springs = num_springs * (MAX_TIME / sim.getDeltaT());
		float springs_per_sec = num_springs / execute_time;

		fw << num_springs << "," << execute_time << "," << springs_per_sec << "\n";
		printf("%u ROBOTS (%.2e SPRINGS) IN %f SECONDS\n", pop_size, (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
}

void DevoBenchmark() {
	printf("BENCHMARKING VOXEL DEVOS\n");
	VoxelRobot R;

	uint pop_size = INIT_POP_SIZE;

	Config::Simulator simConfig;
	ulong num_springs = simConfig.replaced_springs_per_element;
	sim.Initialize(simConfig);

	fw.Open(out_dir + "/devo_benchmark.csv");
	fw << "springs replaced, execute time, springs per second\n";

	float execute_time;


	while(pop_size < MAX_ROBOTS*20) {
		std::vector<Element> robots;
		for(uint i = 0; i < pop_size; i++) {
			robots.push_back(R);
		}
		printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		

		sim.SetElements(robots);

		printf("STARTED\n");
		auto start = std::chrono::high_resolution_clock::now();
		
		sim.Devo();
		auto end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = simConfig.replaced_springs_per_element * pop_size;
		float springs_per_sec = num_springs / execute_time;

		fw << num_springs << "," << execute_time << "," << springs_per_sec << "\n";
		printf("%.2e SPRINGS REPLACED IN %f SECONDS\n", (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
}

void NNBuildBenchmark() {
	printf("BENCHMARKING NN BUILD\n");

	NNRobot R;

	uint pop_size = INIT_POP_SIZE;

	fw.Open(out_dir + "/nnbuild_benchamrk.csv");
	fw << "springs simulated, execute time, springs per second\n";

	float execute_time;

	ulong num_springs;
	while(pop_size <= MAX_ROBOTS) {
		std::vector<NNRobot> robots;

		auto start = std::chrono::high_resolution_clock::now();
		for(uint i = 0; i < pop_size; i++) {
			NNRobot R;
			R.Randomize();
			robots.push_back(R);
		}
		NNRobot::BatchBuild(robots);
		auto end = std::chrono::high_resolution_clock::now();

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = 0;
		for(auto& R : robots) {
			num_springs += R.getSprings().size();
		}
		float springs_per_sec = num_springs / execute_time;

		fw << num_springs << "," << execute_time << "," << springs_per_sec << "\n";
		printf("BUILT %u ROBOTS IN %f SECONDS\n", pop_size, execute_time);
		printf("%.2e SPRINGS PER SECONDS\n", springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	}
}

void NNBenchmark() {
	printf("BENCHMARKING NN\n");

	NNRobot R;
	ulong num_springs;
	
	uint pop_size = INIT_POP_SIZE;

	fw.Open(out_dir + "/nn_benchamrk.csv");
	fw << "springs simulated, execute time, springs per second\n";

	float execute_time;

	do {
		std::vector<NNRobot> robots;
		std::vector<Element> robot_elements;

		auto start = std::chrono::high_resolution_clock::now();
		for(uint i = 0; i < pop_size; i++) {
			NNRobot R;
			R.Randomize();
			robots.push_back(R);
		}
		NNRobot::BatchBuild(robots);
		auto end = std::chrono::high_resolution_clock::now();

		execute_time = std::chrono::duration<float>(end - start).count();
		printf("BUILT %u ROBOTS IN %f SECONDS\n", pop_size, execute_time);

		// NNRobot::BatchBuild(robots);

		for(auto& R : robots) {
			robot_elements.push_back(R);
		}

		
		// printf("POPULATION SIZE:\t%u ROBOTS\n", pop_size);
		
		std::vector<ElementTracker> trackers = sim.SetElements(robot_elements);

		printf("STARTED\n");
		start = std::chrono::high_resolution_clock::now();
		
		sim.Simulate(MAX_TIME);
		end = std::chrono::high_resolution_clock::now();
		printf("FINISHED\n\n");

		std::vector<Element> results = sim.Collect(trackers);

		execute_time = std::chrono::duration<float>(end - start).count();

		num_springs = 0;
		for(auto& R : robot_elements) {
			num_springs += R.springs.size();
		}
		num_springs = num_springs * (MAX_TIME / sim.getDeltaT());
		float springs_per_sec = num_springs / execute_time;

		fw << num_springs << "," << execute_time << "," << springs_per_sec << "\n";
		
		printf("%u ROBOTS (%.2e SPRINGS) IN %f SECONDS\n", pop_size, (float) num_springs, execute_time);
		printf("%.2e SPRINGS PER SECOND\n", (float) springs_per_sec);
		printf("--------------------------\n");

		pop_size *= 2;
	} while(num_springs < MAX_SPRINGS);
}