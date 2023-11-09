#include "Simulator.h"
#include "optimizer_util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"

#include <regex>
#include <thread>
#include <chrono>
// #include <iostream>
#include <string>
#include <sys/stat.h>

#include "opt_tests.h"

uint runID = 0;
uint solID = 0;

Simulator sim;
Config config;

int main(int argc, char** argv)
{
	srand(75);
	std::default_random_engine(75);

	int err = TestEvaluator();
    if(err) {
        std::cout << "Test Case 1: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 1: Passed" << std::endl;
    }

	return 0;
}