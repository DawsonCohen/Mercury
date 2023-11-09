#include "opt_tests.h"

std::vector<float> runEvaluator(std::vector<NNRobot> evalBuf) {
	std::vector<float> fitness(evalBuf.size());

    Evaluator<NNRobot>::BatchEvaluate(evalBuf);

	for(uint i = 0; i < evalBuf.size(); i++) {
		fitness[i] = evalBuf[i].fitness();
	}

	return fitness;
}