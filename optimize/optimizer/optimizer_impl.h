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
#include "optimizer_util.h"

template<typename T>
Optimizer<T>::Optimizer() {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
    gen = std::default_random_engine(seed);
    uniform_real = std::uniform_real_distribution<>(0.0,1.0);
    gamma = std::gamma_distribution<>(1,5);
}

template<typename T>
Optimizer<T>::~Optimizer() {
    
}

template<typename T>
void Optimizer<T>::WriteSolutions(const std::vector<T>& solutions, const std::string& directory) {
    uint i = 0;

    for(const T& R : solutions) {
        std::string encoding = R.Encode();
        float fitness = R.fitness();
        std::string filename = std::string("solution_") + std::to_string(i);
        filename = filename + "_fintess_" + std::to_string(fitness);
        
        util::WriteCSV(filename,directory,encoding);
        i++;
    }
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

    std::vector<T> evalBuf;
    for(uint i = 0; i < population.size(); i++) {
        evalBuf.push_back(population[i]);
    }
    
    Evaluator<T>::BatchEvaluate(evalBuf);

    for(uint i = 0; i < population.size(); i++) {
        population[i] = evalBuf[i];
    }
}

template<typename T>
void Optimizer<T>::RandomizeSolution(Solution<T> sol) {
    sol->Randomize();
    sol->IncrementAge();
}

template<typename T>
void Optimizer<T>::MutateSolution(Solution<T> sol) {
    sol->Mutate();
    sol->IncrementAge();
}

template<typename T>
size_t SelectRoulette(subpopulation<T>& subpop) {
    std::vector<float> fitness(subpop.size());
    float tot_fitness = 0;
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        tot_fitness += i->fitness();
    }
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        fitness[i-subpop.begin()] = i->fitness() / tot_fitness;
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

    for(uint i = 0; i < num_children; i ++) {
        if(uniform_real(gen) >= mutation_crossover_threshold) {
            if(crossover == CROSS_NONE) continue;

            CandidatePair<T> children;
            SolutionPair<T> parents;
            size_t first, second;
            
            first = SelectRoulette(subpop);
            do {
                second = SelectRoulette(subpop);
            } while(first != second);

            parents.first   = &subpop[first];
            parents.second  = &subpop[second];

            subpop[first].setParent(1);
            subpop[second].setParent(1);

            children = T::Crossover({*parents.first, *parents.second});
            
            subpop.crossoverFamilyBuffer.push_back({parents, children});
        } else {
            int working_index = uniform_int(gen);
            Solution<T> working_sol = &subpop[working_index];
            T new_sol(*working_sol);

            switch(mutator){
                case MUTATE_RANDOM:
                    subpop[working_index].setParent(1);
                    RandomizeSolution(&new_sol);
                    break;
                case MUTATE:
                default:
                {
                    subpop[working_index].setParent(1);
                    MutateSolution(&new_sol);
                }
            }
            AsexualFamily<T> fam{working_sol, new_sol};
            subpop.mutationFamilyBuffer.push_back(fam);            
        }
    }

    //STEP 2: Evaluate Children
    std::vector<T> evalBuf;

    for(AsexualFamily<T>& fam : subpop.mutationFamilyBuffer) {
        evalBuf.push_back(fam.child);
    }

    for(SexualFamily<T>& fam : subpop.crossoverFamilyBuffer) {
        evalBuf.push_back(fam.children.first);
        evalBuf.push_back(fam.children.second);
    }

    Evaluator<T>::BatchEvaluate(evalBuf);

    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        evalBuf.push_back(*i);
    }

    Evaluator<T>::pareto_classify(evalBuf.begin(),evalBuf.end());

    uint count = 0;
    for(AsexualFamily<T>& fam : subpop.mutationFamilyBuffer) {
        fam.child = evalBuf[count++];
    }

    for(SexualFamily<T>& fam : subpop.crossoverFamilyBuffer) {
        fam.children.first = evalBuf[count++];
        fam.children.second = evalBuf[count++];
    }

    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        *i = evalBuf[count++];
    }

    //STEP 3: Compare Children to Parents
    int max_elite = elitism * subpop.size();

    for(AsexualFamily<T>& fam : subpop.mutationFamilyBuffer) {
        uint r_idx = max_elite + (rand() % (subpop.size()-max_elite));
        Solution<T> random_solution  = &subpop[r_idx];
        // while(random_solution->paretoLayer() == 0) {
        //     r_idx = max_elite + (rand() % (subpop.size()-max_elite));
        //     random_solution  = &subpop[r_idx];
        // }
        if(max_elite > 0) assert(r_idx != 0);
        if(*random_solution < fam.child) {
            // swap(*(fam.parent), fam.child);
            swap(*(random_solution), fam.child);
        }
    }
    subpop.mutationFamilyBuffer.clear();

    for(SexualFamily<T>& fam : subpop.crossoverFamilyBuffer) {
        switch(crossover) {
            case CROSS_DC: 
            {
                float D00, D11, D01, D10;
                CandidatePair<T> P0 = {fam.children.first, *fam.parents.first},
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
                SolutionPair<T> random;
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
    std::vector<float> diversity = T::findDiversity(population);

    population_history.push_back({Evaluator<T>::eval_count, generation_history, diversity});
    while(Evaluator<T>::eval_count < max_evals) {

        ChildStep(full_pop);

        for(uint i = 0; i < population.size(); i++) {
            if(full_pop[i].isParent()){
                population[i].IncrementAge();
                full_pop[i].setParent(0);
            }
        }
        Evaluator<T>::pareto_sort(population.begin(),population.end());

        if(generation%injection_rate == 0){
            RandomizeSolution(&population[population.size()-1]);
            Evaluator<T>::pareto_sort(population.begin(),population.end());
        }
        diversity = T::findDiversity(population);

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

        for(i = 0; i < population.size(); i++)
            generation_history[i] = population[i].fitness();
        population_history.push_back({Evaluator<T>::eval_count, generation_history, diversity});

        printf("Generation: %lu, Evlauation: %lu\t%s\n",
            generation,
            Evaluator<T>::eval_count,
            population[0].fitnessReadout().data());
        printf("----PARETO SOLUTIONS----\n");
        
        i = 0;
        T sol;
        pareto_solutions.clear();
        float best_fitness = -1000.0f;
        while(i < pop_size) {
            if(population[i].paretoLayer() > 0) break;
            sol = population[i];
            pareto_solutions.push_back(sol);
            if(sol.fitness() > best_fitness) best_fitness = sol.fitness();
            printf("%u:\t%s\n",
                i,
                population[i].fitnessReadout().data());
            i++;
        }
        uint valid = 0;
        uint invalid = 0;
        for(const Candidate& c : population) {
            if(c.isValid()) valid++;
            else invalid++;
        }
        printf("Valid: %u,\tInvalid: %u\n",valid,invalid);
        printf("----------------------\n");

        std::string gen_directory = working_directory + "/generation_" + std::to_string(generation) + "_fitness_" + std::to_string(best_fitness);
        WriteSolutions(pareto_solutions,gen_directory);
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
std::vector<T> Optimizer<T>::Solve(OptimizerConfig config) {
    OptimizerConfig::Optimizer opt_config = config.optimizer;
    niche_count = opt_config.niche_count;
    steps_to_combine = opt_config.steps_to_combine;
    exchange_steps = opt_config.steps_to_exchange;
    max_evals = opt_config.max_evals;
    pop_size = opt_config.pop_size;
    mutator = opt_config.mutation;
    crossover = opt_config.crossover;
    niche = opt_config.niche;
    uniform_int = std::uniform_int_distribution<>(0,pop_size-1);


    elitism = opt_config.elitism;

    for(int N = 0; N < opt_config.repeats; N++) {
        printf("Started Run %i\n",N);

		working_directory = std::string(config.io.out_dir) + std::string("/run_") + std::to_string(N);
        util::MakeDirectory(working_directory);

        switch(niche){
            case NICHE_NONE:
                solutions = NoNicheSolve();
                break;
            default:
                solutions = NoNicheSolve();
                break;
        }

		printf("SOLUTIONS: %lu\n", solutions.size());

		std::string solution_fitness_filename("solution_history.csv");
		std::string population_fitness_filename("fitness_history.csv");
		std::string population_diversity_filename("diversity_history.csv");

		std::string fitnessHistoryCSV = util::FitnessHistoryToCSV(getFitnessHistory());
		std::string popFitHistoryCSV = util::PopulationFitnessHistoryToCSV(getPopulationHistory());
		std::string popDivHistoryCSV = util::PopulationDiversityHistoryToCSV(getPopulationHistory());

		util::WriteCSV(solution_fitness_filename, working_directory, fitnessHistoryCSV);
		util::WriteCSV(population_fitness_filename, working_directory, popFitHistoryCSV);
		util::WriteCSV(population_diversity_filename, working_directory, popDivHistoryCSV);
        
		printf("Run %i Success\n", N);
		reset();
	}

    return solutions;
}

#endif