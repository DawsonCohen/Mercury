#ifndef __TESTS_H__
#define __TESTS_H__

#include <vector>
#include "Simulator.h"
#include "SoftBody.h"

std::vector<float> runSimulator(Simulator& sim, std::vector<SoftBody> robots, float simTime, bool trace = false);
std::vector<float> runSimulator(Simulator& sim, std::vector<SoftBody> robots, float simTime, uint devoCycles, float devoTime, bool trace = false);

int TestSimulator();
int TestNNRobot();
int TestIntegrated();
int TestDevo();


#endif