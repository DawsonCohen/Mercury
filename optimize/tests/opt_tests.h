#ifndef __OPT_TESTS_H__
#define __OPT_TESTS_H__

#include <vector>
#include "NNRobot.h"
#include "Evaluator.h"

std::vector<float> runEvaluator(std::vector<NNRobot> evalBuf);

int TestEvaluator();

#endif