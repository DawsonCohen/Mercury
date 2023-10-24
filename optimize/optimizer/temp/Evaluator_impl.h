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
void Evaluator<T>::Initialize(OptimizerConfig config, T prototype) {
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
    std::vector<float> og_fitness(solutions.size());
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
    
    // for(uint i = 0; i < devoCycles; i++) {
    //     Sim.Simulate(devoTime, true);

    //     Sim.Devo();
    // }

    // Sim.Simulate(baselineTime);
    // std::vector<Element> results = Sim.Collect(trackers);

    // for(uint i = 0; i < solutions.size(); i++) {
    //     solutions[i].Update(results[i]);
    //     solutions[i].updateBaseline();
    // }
    
    Sim.Simulate(evaluationTime);
    std::vector<Element> results = Sim.Collect(trackers);

    uint skip_count = 0;
    for(uint i = 0; i < solutions.size(); i++) {
        if(robotWasAllocated[i]) {
            solutions[i].Update(results[i - skip_count]);
            solutions[i].updateBaseline();
            og_fitness[i] = solutions[i].fitness();
        } else {
            skip_count++;
        }
    }

    Sim.Reset();

    elements.clear();
    for(auto& R : solutions) {
        R.Reset();
        if(R.isValid()) elements.push_back(R);
    }

    trackers = Sim.SetElements(elements);

    // Sim.Simulate(baselineTime);
    // results = Sim.Collect(trackers);

    // for(uint i = 0; i < copy_solutions.size(); i++) {
    //     copy_solutions[i].Update(results[i]);
    //     copy_solutions[i].updateBaseline();
    // }
    
    Sim.Simulate(evaluationTime);
    results = Sim.Collect(trackers);

    skip_count = 0;
    for(uint i = 0; i < solutions.size(); i++) {
        if(robotWasAllocated[i]) {
            solutions[i].Update(results[i - skip_count]);
            solutions[i].updateBaseline();
            printf("og fitness: %f, new fitness: %f\n", og_fitness[i], solutions[i].fitness());
        } else {
            skip_count++;
        }
    }

    eval_count += solutions.size();

    // std::vector<T> good_solutions;
    // std::vector<Element> good_elements;
    // std::vector<float> og_fitness;
    // std::vector<float> base_fitness;

    // for(auto& R : solutions) {
    //     if(R.fitness() > 0.2f) {
    //         og_fitness.push_back(R.fitness());
            
    //         good_solutions.push_back(R);
    //         good_solutions[good_solutions.size()-1].updateBaseline();

    //         good_elements.push_back({R.getMasses(), R.getSprings()});
    //     }
    // }
    // if(good_solutions.size() == 0) {
    //     Sim.Reset();
    //     return;
    // }
    // std::vector<ElementTracker> good_trackers = Sim.SetElements(good_elements);

    // Sim.Simulate(evaluationTime);
    // std::vector<Element> good_results = Sim.Collect(trackers);

    // for(uint i = 0; i < good_solutions.size(); i++) {
    //     good_solutions[i].Update(good_results[i]);
    //     base_fitness.push_back(good_solutions[i].fitness());
    // }
    
    // Sim.Reset();
    // good_elements.clear();
    // for(auto& R : good_solutions) {
    //     R.Reset();
    //     good_elements.push_back({R.getMasses(), R.getSprings()});
    // }
    
    // good_trackers = Sim.SetElements(good_elements);
    // Sim.Simulate(baselineTime);
    // good_results = Sim.Collect(trackers);

    // for(uint i = 0; i < good_solutions.size(); i++) {
    //     good_solutions[i].Update(good_results[i]);
    //     good_solutions[i].updateBaseline();
    // }
    
    // Sim.Simulate(evaluationTime);
    // good_results = Sim.Collect(trackers);

    // for(uint i = 0; i < good_solutions.size(); i++) {
    //     good_solutions[i].Update(good_results[i]);
    //     printf("og fitness: %f, base fitness %f, new reset fitness: %f\n", og_fitness[i], base_fitness[i], good_solutions[i].fitness());
    // }
    // printf("\n");

    Sim.Reset();
}

#endif