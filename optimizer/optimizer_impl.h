#ifndef __Optimizer_Impl_H__
#define __Optimizer_Impl_H__

#include "optimizer.h"
// #include "cuda_runtime.h"
// #include "device_launch_parameters.h"
// #include <curand.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <thread>
#include <functional>
#include <cassert>
#include <time.h>
#include <memory>
#include <cmath>
#include <utility>
//#include "util.h"

template<typename T>
Optimizer<T>::Optimizer() {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
    gen = std::default_random_engine(seed);
    uniform = std::uniform_real_distribution<>(0.0,1.0);
    gamma = std::gamma_distribution<>(1,5);
}

template<typename T>
Optimizer<T>::~Optimizer() {
    
}

template<typename T>
std::vector<float> updateDiversity(std::vector<T> pop) {
    return T::findDiversity(pop);
}

template<typename T>
void Optimizer<T>::RandomizePopulation(std::vector<T>& population) {
    printf("RANDOMIZING\n");
    for(uint i = 0; i < population.size(); i++) {
        population[i].Randomize();
    }

    std::vector<T> evalBuf(population.size());
    for(uint i = 0; i < population.size(); i++)
        evalBuf[i] = population[i];
    
    Evaluator<T>::BatchEvaluate(evalBuf);

    for(uint i = 0; i < population.size(); i++) {
        population[i] = evalBuf[i];
    }
}

template<typename T>
T Optimizer<T>::RandomizeSolution(T& working_sol) {
    T new_sol = working_sol;

    new_sol.Randomize();
    new_sol.mAge = 1;

    return new_sol;
}

template<typename T>
T Optimizer<T>::MutateSolution(T& working_sol) {
    T new_sol = working_sol;
    new_sol.Mutate();
    new_sol.mAge += 1;

    return new_sol;
}

template<typename T>
T& SelectRoulette(subpopulation<T>& subpop) {
    std::vector<float> fitness(subpop.size());
    float tot_fitness = 0;
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        tot_fitness += 1/(-i->fitness());
    }
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        fitness[i-subpop.begin()] = (1/(-i->fitness())) / tot_fitness;
    }
    
    float random = ((float) rand() / (float) RAND_MAX);

    size_t i = 0;
    while(i < fitness.size()-1) {
        random -= fitness[i];
        if(random < 0) break;

        i++;
    }
    
    return i;
}

template<typename T>
void Optimizer<T>::ChildStep(subpopulation<T>& subpop) {
    //STEP 1: Generate new population of children
    uint num_children = subpop.size() * child_pop_size;

    for(int i = 0; i < num_children; i ++) {
        if(uniform(gen) >= mutation_crossover_threshold) {
            if(crossover == CROSS_NONE) continue;

            CandidatePair<T> children;
            SolutionPair<T> parents;
            size_t first, second;

            first = SelectRoulette(subpop);
            do {
                second = SelectRoulette(subpop);
            } while(first != second)

            parents.first   = (subpop.begin()+first);
            parents.second  = (subpop.begin()+second);

            subpop.parentFlag[first] = 1;
            subpop.parentFlag[second] = 1;

            CandidatePair<T> p = {*parents.first, *parents.second};

            children = T::Crossover(p);
            
            subpop.crossoverFamilyBuffer.push_back({parents, children});
        } else {
            T new_sol;
            switch(mutator){
                case RANDOMIZE:
                    i->IncrementAge();
                    new_sol = RandomizeSolution(*i);
                    break;
                case MUTATE:
                default:
                {
                    float rand = uniform(gen);
                    if(rand > mutation_rate) continue;
                    subpop.parentFlag[it - vec.begin()] = 1
                    new_sol = MutateSolution(*i);
                }
            }
            AsexualRobotFamily fam{i.base(), new_sol};
            subpop.mutationFamilyBuffer.push_back(fam);            
        }
    }

    //STEP 2: Evaluate Children
    std::vector<T> evalBuf;

    for(AsexualRobotFamily& fam : subpop.mutationFamilyBuffer) {
        evalBuf.push_back(fam.child);
    }

    for(SexualRobotFamily& fam : subpop.crossoverFamilyBuffer) {
        evalBuf.push_back(fam.children.first);
        evalBuf.push_back(fam.children.second);
    }

    Evaluator<T>::BatchEvaluate(evalBuf);

    for(T& R : subpop.population) {
        evalBuf.push_back(R);
    }

    Evaluator<T>::pareto_classify(evalBuf.begin(),evalBuf.end());

    uint count = 0;
    for(AsexualRobotFamily& fam : subpop.mutationFamilyBuffer) {
        fam.child = evalBuf[count++];
    }

    for(SexualRobotFamily& fam : subpop.crossoverFamilyBuffer) {
        fam.children.first = evalBuf[count++];
        fam.children.second = evalBuf[count++];
    }

    for(T& R : subpop.population) {
        R = evalBuf[count++];
    }

    //STEP 3: Compare Children to Parents
    int max_elite = elitism * subpop.size();

    for(AsexualRobotFamily& fam : subpop.mutationFamilyBuffer) {
        uint r_idx = max_elite + (rand() % (subpop.size()-max_elite));
        T* random_robot  = &subpop[r_idx];
        // while(random_robot->paretoLayer() == 0) {
        //     r_idx = max_elite + (rand() % (subpop.size()-max_elite));
        //     random_robot  = &subpop[r_idx];
        // }
        assert(r_idx != 0);
        if(*random_robot <= fam.child) {
            swap(*(fam.parent), fam.child);
        }
    }
    subpop.mutationFamilyBuffer.clear();

    for(SexualRobotFamily& fam : subpop.crossoverFamilyBuffer) {
        switch(crossover) {
            case CROSS_DC: 
            {
                float D00, D11, D01, D10;
                RobotPair P0 = {fam.children.first, *fam.parents.first},
                              P1 = {fam.children.second, *fam.parents.second},
                              P2 = {fam.children.first,  *fam.parents.second},
                              P3 = {fam.children.second, *fam.parents.first };


                D00 = T::Distance(P0);
                D11 = T::Distance(P1);
                D01 = T::Distance(P2);
                D10 = T::Distance(P3);

                if(D00 + D11 < D01 + D10) {
                    if(fam.children.first >= *fam.parents.first) swap(fam.children.first,*fam.parents.first);
                    if(fam.children.second >= *fam.parents.second) swap(fam.children.second,*fam.parents.second);
                } else {
                    if(fam.children.first >= *fam.parents.second) swap(fam.children.first,*fam.parents.second);
                    if(fam.children.second >= *fam.parents.first) swap(fam.children.second,*fam.parents.first);
                }
            }
            break;
            case CROSS_SWAP:
            default:
            {
                SolutionPair random;
                uint r1 = max_elite + (rand() % (subpop.size()-max_elite));
                uint r2 = max_elite + (rand() % (subpop.size()-max_elite));
                
                random.first  = &subpop[r1];
                random.second = &subpop[r2];

                if(fam.children.first  >= *random.first) swap(fam.children.first,  *random.first);
                if(fam.children.second >= *random.first) swap(fam.children.second, *random.first);

                if(fam.children.first  >= *random.second) swap(fam.children.first,  *random.second);
                if(fam.children.second >= *random.second) swap(fam.children.second, *random.second);
            }
        }
    }
    subpop.crossoverFamilyBuffer.clear();
}

template<typename T>
std::vector<T> Optimizer<T>::NoNicheSolve() {
    niche_count = thread_count;
    subpop_size = pop_size/niche_count;
    
    std::vector<std::thread*> threads(niche_count);
    std::vector<T> population(pop_size);
    subpop_list = std::vector<subpopulation<T>>(niche_count);
    subpopulation<T> full_pop(population.begin(), population.end());

    if(subpop_size == 0) {
        niche_count = 1;
        subpop_size = pop_size;
        auto begin = population.begin();
        auto end = population.end();
        subpop_list[0] = subpopulation<T>(begin, end);
    } else {
        for(uint i = 0; i < niche_count; i++) {
            auto begin = population.begin() + i*subpop_size;
            auto end = begin + subpop_size;
            subpop_list[i] = subpopulation<T>(begin, end);
        }
    }

    RandomizePopulation(population);
    Evaluator<T>::pareto_sort(population.begin(),population.end());
    printf("Initial Best: %f\n",population[0].fitness());

    ulong generation = 1;
    std::vector<float>generation_history(population.size());
    for(size_t i = 0; i < population.size(); i++)
        generation_history[i] = population[i].fitness();
    std::vector<float> diversity = updateDiversity(population);
    population_history.push_back({Evaluator<T>::eval_count, generation_history, diversity});

    while(Evaluator<T>::eval_count < max_evals) {

        ChildStep(full_pop)

        for(uint i = 0; i < population.size(); i++) {
            if(full_pop.parentFlag[i] == 1){
                population[i].IncrementAge();
                full_pop.parentFlag[i] == 0;
            }
        }
        Evaluator<T>::pareto_sort(population.begin(),population.end());

        if(generation%injection_rate == 0){
            population[population.size()-1] = RandomizeSolution(population[population.size()-1])
            Evaluator<T>::pareto_sort(population.begin(),population.end());
        }

        diversity = updateDiversity(population);

        T archived_solution;
        archived_solution = population[0];
        uint i = 1;
        while (population[i] <= population[i+1] && i < population.size())
        {
            if(archived_solution.fitness() < population[i].fitness())
                archived_solution = population[i];
            i++;
        }

        solution_history.push_back({Evaluator<T>::eval_count, archived_solution});
        fitness_history.push_back({Evaluator<T>::eval_count, archived_solution.fitness()});
        diversity_history.push_back({Evaluator<T>::eval_count, diversity[0]});

        //generation_digits = floor(log10(generation));
        if(generation % print_skip == 0) {
        // if(generation/pow(hist_skip_factor,generation_digits) == 1) {
            for(size_t i = 0; i < population.size(); i++)
                generation_history[i] = population[i].fitness();
            population_history.push_back({Evaluator<T>::eval_count, generation_history, diversity});
        }

        if(generation % print_skip == 0) {
            printf("Generation: %lu, Evlauation: %lu\t%s\n",
                generation,
                Evaluator<T>::eval_count,
                population[0].fitnessReadout().data());
            printf("----PARETO SOLUTIONS----\n");
            uint i = 0;
            T sol;
            while(i < pop_size) {
                if(population[i].paretoLayer() > 0) break;
                sol = population[i];
                printf("%u:\t%s\n",
                    i,
                    population[i].fitnessReadout().data());
                i++;
            }
            printf("----------------------\n");
        }
        generation++;
    }

    Evaluator<T>::pareto_sort(population.begin(),population.end());

    solutions.clear();
    solutions.push_back(population[0]);
    uint i = 1;
    while(i < pop_size) {
        if(population[i].paretoLayer() > 0) break;
        solutions.push_back(population[i]);
        i++;
    }
    return solutions;
}

template<typename T>
std::vector<T> Optimizer<T>::Solve() {
    switch(niche){
        case NICHE_NONE:
            return NoNicheSolve();
            break;
        default:
            return NoNicheSolve();
            break;
    }
}

#endif