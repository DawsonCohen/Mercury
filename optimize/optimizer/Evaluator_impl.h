#ifndef __Evaluator_Impl_H__
#define __Evaluator_Impl_H__

#include "Evaluator.h"

template<typename T>
ulong Evaluator<T>::eval_count = 0;

template<typename T>
float Evaluator<T>::baselineTime = 5.0f;

template<typename T>
float Evaluator<T>::evaluationTime = 10.0f;

template<typename T>
Config::Simulator Evaluator<T>::sim_config = Config::Simulator();

template<typename T>
Simulator Evaluator<T>::Sim = Simulator();

template<typename T>
void Evaluator<T>::Initialize(Config config) {
    T prototype;
    
    sim_config = config.simulator;
	Sim.Initialize(prototype, config.evaluator.pop_size*1.5, sim_config);
    baselineTime = config.evaluator.base_time;
    evaluationTime = config.evaluator.eval_time;
}

template<typename T>
void Evaluator<T>::BatchEvaluate(std::vector<T>& solutions) {
    if(solutions.size() == 0) return;
    printf("EVALUATING %lu SOLUTIONS\n",solutions.size());

    std::vector<Element> elements;

    for(auto& R : solutions) {
        R.Reset();
        if(R.isValid())
            elements.push_back({R.getMasses(), R.getSprings()});
    }
    
    Sim.setMaxTime(baselineTime);

    std::vector<ElementTracker> trackers = Sim.Simulate(elements);
    
    std::vector<Element> results = Sim.Collect(trackers);

    for(uint i = 0; i < solutions.size(); i++) {
        solutions[i].Update(results[i]);
        solutions[i].updateBaseline();
    }
    Sim.setMaxTime(evaluationTime);
    // TODO: "continue" simulation without copying vector again
    trackers = Sim.Simulate(elements);
    results = Sim.Collect(trackers);

    for(uint i = 0; i < solutions.size(); i++) {
        solutions[i].Update(results[i]);
    }

    for(T& R : solutions) {
        eval_count++;
        R.updateFitness();
    }

    Sim.Reset();
}

#endif