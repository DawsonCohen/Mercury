#include "Simulator.h"
#include "test_evaluator.h"
#include "NNRobot.h"
#include "VoxelRobot.h"

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

uint runID = 0;
uint solID = 0;

int handle_file_io();

Simulator sim;
Config config;

int main(int argc, char** argv)
{
	srand(75);
	std::default_random_engine(75);
	// handle_file_io();

	int err = TestEvaluator();
    if(err) {
        std::cout << "Test Case 1: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 1: Passed" << std::endl;
    }

	return 0;
}

int handle_file_io() {
	// Get current date and time
	auto now = std::chrono::system_clock::now();
	time_t current_time = std::chrono::system_clock::to_time_t(now);

	// Create config out_dir folder
	util::MakeDirectory(config.io.out_dir);

	// Create folder with current time as name
	config.io.out_dir = config.io.out_dir + "/unit_tests";
	util::MakeDirectory(config.io.out_dir);

	return 0;
}