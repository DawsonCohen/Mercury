#include "EvoDevo.h"

#include "EvoDevo/Simulator/Simulator.h"
#include "EvoDevo/Util/util.h"

#include "tests.h"

using namespace EvoDevo;

Config config;

int main(int argc, char** argv)
{
	int err = 0;

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

    err = TestNNBuild();
	if(err) {
        std::cout << "Test Case 3: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 3: Passed" << std::endl;
    }

    err = TestIntegrated();
    if(err) {
        std::cout << "Test Case 4: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 4: Passed" << std::endl;
    }

	err = TestDevo();
    if(err) {
        std::cout << "Test Case 5: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 5: Passed" << std::endl;
    }

    err = TestTransfer();
    if(err) {
        std::cout << "Test Case 6: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 6: Passed" << std::endl;
    }

    err = TestMatEncoding();
	if(err) {
        std::cout << "Test Case 7: Failed with " << err << std::endl;
    } else {
        std::cout << "Test Case 7: Passed" << std::endl;
    }

	return 0;
}
