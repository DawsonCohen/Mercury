#include "Simulator.h"
#include "util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>

#include <regex>
#include <thread>
#include <chrono>
#include <string>
#include <sys/stat.h>

#include "common_tests.h"

Config config;

int main(int argc, char** argv)
{
	std::vector<NNRobot> robots;
	int err;

	err = TestSimulator();
    if(err) {
        std::cout << "Test Case 1: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 1: Passed" << std::endl;
    }

	err = TestNNRobot();
	if(err) {
        std::cout << "Test Case 2: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 2: Passed" << std::endl;
    }

    err = TestIntegrated();
    if(err) {
        std::cout << "Test Case 3: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 3: Passed" << std::endl;
    }

	err = TestDevo();
    if(err) {
        std::cout << "Test Case 4: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 4: Passed" << std::endl;
    }

    err = TestTransfer();
    if(err) {
        std::cout << "Test Case 5: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 5: Passed" << std::endl;
    }

    err = TestMatEncoding();
	if(err) {
        std::cout << "Test Case 6: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 6: Passed" << std::endl;
    }

	return 0;
}