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
#include "util.h"

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
void Optimizer<T>::MutateStep(subpopulation<T>& subpop) {
    // TODO: Don't include non-mutated iterations
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        T new_sol;
        switch(mutator){
            case MUTATE_RANDOM:
                new_sol = RandomizeSolution(*i);
                break;
            case MUTATE:
            default:
            {
                float rand = uniform(gen);
                if(rand > mutation_rate) continue;
                new_sol = MutateSolution(*i);
            }
        }
        i->IncrementAge();
        AsexualFamily<T> fam{i.base(), new_sol};
        subpop.mutationFamilyBuffer.push_back(fam);
    }
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
    
    return *(subpop.begin()+i);
}

template<typename T>
void Optimizer<T>::CrossoverStep(subpopulation<T>& subpop) {
    uint num_replaced = subpop.size() * crossover_rate;

    if(crossover == CROSS_NONE) return;

    CandidatePair<T> children;
    for(uint i = 0; i < num_replaced; i++) {
        SolutionPair<T> parents;

        parents.first   = &SelectRoulette(subpop);
        parents.second  = &SelectRoulette(subpop);

        parents.first->IncrementAge();
        parents.second->IncrementAge();

        CandidatePair<T> p = {*parents.first, *parents.second};

        children = T::Crossover(p);
        
        subpop.crossoverFamilyBuffer.push_back({parents, children});
    }
}

template<typename T>
void Optimizer<T>::MutateCollect(std::vector<subpopulation<T>>& subpop_list) {
    std::vector<T> evalBuf;

    for(subpopulation<T>& subpop : subpop_list) {
        for(AsexualFamily<T>& fam : subpop.mutationFamilyBuffer) {
            evalBuf.push_back(fam.child);
        }
    }
    
    Evaluator<T>::BatchEvaluate(evalBuf);

    uint count = 0;
    for(subpopulation<T>& subpop : subpop_list) {
        for(AsexualFamily<T>& fam : subpop.mutationFamilyBuffer) {
            fam.child = evalBuf[count++];
        }
        Evaluator<T>::pareto_sort(subpop.begin(),subpop.end());
    }

    for(subpopulation<T>& subpop : subpop_list) {
        int max_elite = elitism * subpop.size();

        for(AsexualFamily<T>& fam : subpop.mutationFamilyBuffer) {
            uint r_idx = max_elite + (rand() % (subpop.size()-max_elite));
            T* random_solution  = &subpop[r_idx];

            // while(random_solution->paretoLayer() == 0) {
            //     r_idx = max_elite + (rand() % (subpop.size()-max_elite));
            //     random_solution  = &subpop[r_idx];
            // }
            
            // assert(r_idx != 0);
            if(*random_solution <= fam.child) {
                swap(*(fam.parent), fam.child);
            }
        }
        subpop.mutationFamilyBuffer.clear();
    }
}

template<typename T>
void Optimizer<T>::CrossoverCollect(std::vector<subpopulation<T>>& subpop_list) {
    std::vector<T> evalBuf;

    for(subpopulation<T>& subpop : subpop_list) {
        for(SexualFamily<T>& fam : subpop.crossoverFamilyBuffer) {
            evalBuf.push_back(fam.children.first);
            evalBuf.push_back(fam.children.second);
        }
    }

    Evaluator<T>::BatchEvaluate(evalBuf);

    uint count = 0;
    for(subpopulation<T>& subpop : subpop_list) {
        for(SexualFamily<T>& fam : subpop.crossoverFamilyBuffer) {
            fam.children.first = evalBuf[count++];
            fam.children.second = evalBuf[count++];
        }
        Evaluator<T>::pareto_sort(subpop.begin(),subpop.end());
    }

    for(subpopulation<T>& subpop : subpop_list) {
        int max_elite = elitism * subpop.size();
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
}

template<typename T>
void Optimizer<T>::AlpsMutateStep(subpopulation<T>& subpop) {
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        T new_sol;
        switch(mutator){
            case MUTATE_RANDOM:
                new_sol = RandomizeSolution(*i);
                break;
            case MUTATE:
            default:
            {
                float rand = uniform(gen);
                if(rand > mutation_rate) continue;
                new_sol = MutateSolution(*i);
            }
        }
        AsexualFamily<T> fam{i.base(), new_sol};
        subpop.mutationFamilyBuffer.push_back(fam);
    }
}

template<typename T>
std::vector<T> Optimizer<T>::ALPSSolve() {    
    subpop_size = pop_size/niche_count;

    float export_thresh[] = {2,3,5,8,13,21,34,10000};
    
    std::vector<std::thread*> threads(niche_count);
    subpop_list = std::vector<subpopulation<T>>(niche_count);
    population = std::vector<T>(pop_size);

    if(subpop_size == 0) {
        niche_count = 1;
        subpop_size = pop_size;
        auto begin = population.begin();
        auto end = population.end();
        subpop_list[0] = subpopulation<T>(begin, end);
        subpop_list[0].export_threshold = 10000;
    } else {
        for(uint i = 0; i < niche_count; i++) {
            auto begin = population.begin() + i*subpop_size;
            auto end = begin + subpop_size;
            subpop_list[i] = subpopulation<T>(begin, end);
            subpop_list[i].export_threshold = export_thresh[i];
        }
    }

    RandomizePopulation(population);
    std::sort(population.begin(),population.end(),std::greater<T>());
    printf("Initial Best: %f\n",population[0].fitness());

    ulong generation = 1;
    std::vector<float>generation_history(population.size());
    for(size_t i = 0; i < population.size(); i++)
        generation_history[i] = population[i].fitness();
    std::vector<float> diversity = T::findDiversity(population);
    population_history.push_back({Evaluator<T>::eval_count, generation_history, diversity});

    while(Evaluator<T>::eval_count < max_evals) {
        printf("------MUTATE-----\n");

        for(size_t i = 0; i < subpop_list.size(); i++) {
            MutateStep(subpop_list[i]);
        }

        MutateCollect(subpop_list);

        for(subpopulation<T>& subpop : subpop_list) {
            Evaluator<T>::pareto_sort(subpop.begin(),subpop.end());
        }

        printf("----CROSSOVER----\n");
        for(size_t i = 0; i < niche_count; i++) {
            std::thread* newThread = new std::thread(&Optimizer::CrossoverStep, this, std::ref(subpop_list[i]));
            threads[i] = newThread;
        }
        for(size_t i = 0; i < threads.size(); i++) {
            threads[i]->join();
        }
        for(size_t i = 0; i < threads.size(); i++) {
            delete threads[i];
        }

        for(subpopulation<T>& subpop : subpop_list) {
            Evaluator<T>::pareto_sort(subpop.begin(),subpop.end());
        }

        Evaluator<T>::pareto_sort(population.begin(),population.end());

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
        printf("------MUTATE-----\n");

        for(size_t i = 0; i < subpop_list.size(); i++) {
            MutateStep(subpop_list[i]);
        }

        MutateCollect(subpop_list);

        Evaluator<T>::pareto_sort(population.begin(),population.end());

        printf("----CROSSOVER----\n");
        CrossoverStep(full_pop);

        for(uint i = 0; i < population.size(); i++) {
            population[i].IncrementAge();
        }

        std::vector<subpopulation<T>> temp_pop_list;
        temp_pop_list.push_back(full_pop);
        CrossoverCollect(temp_pop_list);
        full_pop.crossoverFamilyBuffer.clear();
        
        Evaluator<T>::pareto_sort(population.begin(),population.end());

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
std::vector<T> Optimizer<T>::Solve(Config config) {
    Config::Optimzer opt_config = config.optimizer;
    niche_count = opt_config.niche_count;
    steps_to_combine = opt_config.steps_to_combine;
    exchange_steps = opt_config.steps_to_exchange;
    max_evals = opt_config.max_evals;
    pop_size = opt_config.pop_size;
    mutator = opt_config.mutation;
    crossover = opt_config.crossover;
    niche = opt_config.niche;

    mutation_rate = opt_config.mutation_rate;
    crossover_rate = opt_config.crossover_rate;
    elitism = opt_config.elitism;

    for(unsigned N = 0; N < opt_config.repeats; N++) {
        printf("Started Run %i\n",N);

		working_directory = std::string(config.io.out_dir) + std::string("/run_") + std::to_string(N);
        util::MakeDirectory(working_directory);

        switch(niche){
            case NICHE_NONE:
                return NoNicheSolve();
                break;
            default:
                return NoNicheSolve();
                break;
        }

		printf("SOLUTIONS: %lu\n", solutions.size());

		std::string solution_fitness_file("solution_history");
		std::string population_fitness_file("fitness_history");
		std::string population_diversity_file("diversity_history");

		std::string fitnessHistoryCSV = util::FitnessHistoryToCSV(getFitnessHistory());
		std::string popFitHistoryCSV = util::PopulationFitnessHistoryToCSV(getPopulationHistory());
		std::string popDivHistoryCSV = util::PopulationDiversityHistoryToCSV(getPopulationHistory());

		util::WriteCSV(solution_fitness_file, working_directory, fitnessHistoryCSV);
		util::WriteCSV(population_fitness_file, working_directory, popFitHistoryCSV);
		util::WriteCSV(population_diversity_file, working_directory, popDivHistoryCSV);
        
		printf("Run %i Success\n", N);
		reset();
	}

    return solutions;
}

#endif