#ifndef __EVALUATOR_IMPL_H__
#define __EVALUATOR_IMPL_H__

#include "EvoDevo/Optimizer/Evaluator.h"

namespace EvoDevo {

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
    void Evaluator<T>::Initialize(Config config) {
        sim_config = config.simulator;
        baselineTime = config.evaluator.base_time;
        evaluationTime = config.evaluator.eval_time;
        devoTime = config.devo.devo_time;
        devoCycles = config.devo.devo_cycles;
        Sim.Initialize(sim_config);
    }

    template<typename T>
    void Evaluator<T>::BatchEvaluate(std::vector<T>& solutions, bool trace) {
        if(solutions.size() == 0) return;
        printf("EVALUATING %lu SOLUTIONS\n",solutions.size());

        Sim.Reset();

        std::vector<Element> elements;
        std::vector<Element> results;
        uint skip_count;

        std::vector<bool> robotWasAllocated(solutions.size());

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
        results = Sim.Collect(trackers);

        skip_count = 0;
        elements.clear();
        for(uint i = 0; i < solutions.size(); i++) {
            if(robotWasAllocated[i]) {
                solutions[i].Update(results[i - skip_count]);
                solutions[i].Reset();
                elements.push_back(solutions[i]);
            } else {
                skip_count++;
            }
        }
        Sim.Reset();
        trackers = Sim.SetElements(elements); // this can be parallelized!!
        
        static int trace_count = 0;
        Sim.Simulate(evaluationTime, false, trace, std::string("sim_trace_") + std::to_string(trace_count) + std::string(".csv"));
        if(trace) trace_count++;
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
    }  
}

#endif