#ifndef __COMMON_TESTS_H__
#define __COMMON_TESTS_H__

#include <vector>
#include "EvoDevo/Simulator/Simulator.h"
#include "EvoDevo/Evolvables/SoftBody.h"
#include "EvoDevo/Evolvables/NNRobot.h"

namespace EvoDevo {

    std::vector<float> runSimulator(Simulator& sim, std::vector<SoftBody> robots, float simTime, bool trace = false);
    std::vector<float> runSimulator(Simulator& sim, std::vector<SoftBody> robots, float simTime, uint devoCycles, float devoTime, bool trace = false);
    std::vector<float> runEvaluator(std::vector<NNRobot> evalBuf);

    int TestSimulator();
    int TestMatEncoding();
    int TestNNRobot();
    int TestNNBuild();
    int TestIntegrated();
    int TestDevo();
    int TestTransfer();
    int TestEvaluator();

}


#endif