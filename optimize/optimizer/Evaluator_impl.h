#ifndef __EVALUATOR_IMPL_H__
#define __EVALUATOR_IMPL_H__

#include "Evaluator.h"

template<typename T>
ulong Evaluator<T>::eval_count = 0;

template<typename T>
float Evaluator<T>::devoTime = 0;

template<typename T>
float Evaluator<T>::devoCycles = 0;

template<typename T>
float Evaluator<T>::baselineTime = 5.0f;

template<typename T>
float Evaluator<T>::evaluationTime = 10.0f;

template<typename T>
Config::Simulator Evaluator<T>::sim_config = Config::Simulator();

template<typename T>
Simulator Evaluator<T>::Sim = Simulator();


template<typename T>
void Evaluator<T>::Initialize(OptimizerConfig config) {
    sim_config = config.simulator;
    baselineTime = config.evaluator.base_time;
    evaluationTime = config.evaluator.eval_time;
    devoTime = config.devo.devo_time;
    devoCycles = config.devo.devo_cycles;
}

template<typename T>
void Evaluator<T>::BatchEvaluate(std::vector<T>& solutions) {
    if(solutions.size() == 0) return;
    printf("EVALUATING %lu SOLUTIONS\n",solutions.size());

	Sim.Initialize(solutions[0], solutions.size(), sim_config);
    // std::vector<T> copy_solutions;

    std::vector<Element> elements;

    bool robotWasAllocated[solutions.size()];

    uint i = 0;
    for(auto& R : solutions) {
        R.Reset();
        if(R.isValid()) {
            elements.push_back(R);
            robotWasAllocated[i] = true;
        } else {
            robotWasAllocated[i] = false;
        }
        i++;
    }

    std::vector<ElementTracker> trackers = Sim.SetElements(elements);

    
    for(uint i = 0; i < devoCycles; i++) {
        Sim.Simulate(devoTime, true);

        Sim.Devo();
    }

    Sim.Simulate(baselineTime);
    std::vector<Element> results = Sim.Collect(trackers);

    uint skip_count = 0;
    for(uint i = 0; i < solutions.size(); i++) {
        if(robotWasAllocated[i]) {
            solutions[i].Update(results[i - skip_count]);
            solutions[i].updateBaseline();
        } else {
            skip_count++;
        }
    }
    
    Sim.Simulate(evaluationTime);
    results = Sim.Collect(trackers);

    skip_count = 0;
    for(uint i = 0; i < solutions.size(); i++) {
        if(robotWasAllocated[i]) {
            solutions[i].Update(results[i - skip_count]);
        } else {
            skip_count++;
        }
    }

    for(T& R : solutions) {
        eval_count++;
        R.updateFitness();
    }

    Sim.Reset();
}

#endif