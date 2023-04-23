#ifndef __Evaluator_Impl_H__
#define __Evaluator_Impl_H__

#include "Evaluator.h"
#include <iterator>

template<typename T>
ulong Evaluator<T>::eval_count = 0;

template<typename T>
float Evaluator<T>::baselineTime = 5.0f;

template<typename T>
float Evaluator<T>::evaluationTime = 10.0f;

template<typename T>
Simulator Evaluator<T>::Sim = Simulator();

template<typename T>
T Evaluator<T>::protoRobot = T();

template<typename T>
void Evaluator<T>::Initialize(Config config) {
	Sim.Initialize(protoRobot, config.evaluator.pop_size*1.5);
    // Sim.Initialize(prototype, config.evaluator.pop_size*1.5);
    baselineTime = config.evaluator.base_time;
    evaluationTime = config.evaluator.eval_time;
}

template<typename T>
void Evaluator<T>::BatchEvaluate(std::vector<T>& solutions) {
    if(solutions.size() == 0) return;
    printf("EVALUATING %lu SOLUTIONS\n",solutions.size());

    std::vector<Element> elements;

    for(auto& R : solutions) {
        elements.push_back({R.getMasses(), R.getSprings()});
    }
    
    Sim.setMaxTime(baselineTime);

    std::vector<ElementTracker> trackers = Sim.Simulate(elements);
    
    std::vector<Element> results = Sim.Collect(trackers);

    for(uint i = 0; i < solutions.size(); i++) {
        solutions[i].Update(results[i]);
    }

    for(T& e : solutions) {
        e.updateBaseline();
    }

    Sim.setMaxTime(evaluationTime);

    // TODO: "continue" simulation without copying vector again
    trackers = Sim.Simulate(elements);
    results = Sim.Collect(trackers);

    for(uint i = 0; i < solutions.size(); i++) {
        solutions[i].Update(results[i]);

        Eigen::Vector3f COM = T::calcMeanPos(solutions[i]);
    }

    for(T& r : solutions) {
        eval_count++;
        T::calcFitness(r);
        
        r.Reset();
    }

    Sim.Reset();
}

template<typename T>
float Evaluator<T>::Distance(const CandidatePair<T>& solutions) {
    float dist = 0;
    std::vector<Spring> s1 = solutions.first.getSprings();
    std::vector<Spring> s2 = solutions.second.getSprings();
    for(size_t i = 0; i < s1.size(); i++) {
        dist += !(s1[i].material == s2[i].material);
    }
    return dist;
}

#endif