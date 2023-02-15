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

#define max(a,b) a > b ? a : b
#define min(a,b) a < b ? a : b

Optimizer::Optimizer() {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
    gen = std::default_random_engine(seed);
    uniform = std::uniform_real_distribution<>(0.0,1.0);
    gamma = std::gamma_distribution<>(1,5);
}

Optimizer::~Optimizer() {
    
}

std::vector<float> updateDiversity(std::vector<Robot> pop) {
    return Robot::findDiversity(pop);
}

void Optimizer::RandomizePopulation(std::vector<Robot>& population) {
    printf("RANDOMIZING\n");
    for(uint i = 0; i < population.size(); i++) {
        population[i].Randomize();
    }

    std::vector<Robot> evalBuf(population.size());
    for(uint i = 0; i < population.size(); i++)
        evalBuf[i] = population[i];
    
    Evaluator::BatchEvaluate(evalBuf);

    for(uint i = 0; i < population.size(); i++) {
        population[i] = evalBuf[i];
    }
}

Robot Optimizer::RandomizeSolution(Robot& working_sol) {
    Robot new_sol = working_sol;

    new_sol.Randomize();
    new_sol.mAge = 1;

    return new_sol;
}

Robot Optimizer::MutateSolution(Robot& working_sol) {
    Robot new_sol = working_sol;
    new_sol.Mutate();
    new_sol.mAge += 1;

    return new_sol;
}

void Optimizer::MutateStep(subpopulation& subpop) {
    // TODO: Don't include non-mutated iterations
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        Robot new_sol;
        switch(mutator){
            case RANDOMIZE:
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
        AsexualRobotFamily fam{i.base(), new_sol};
        subpop.mutationFamilyBuffer.push_back(fam);
    }
}



Robot& SelectRoulette(subpopulation& subpop) {
    std::vector<float> fitness(subpop.size());
    float tot_fitness = 0;
    for(std::vector<Robot>::iterator i = subpop.begin(); i < subpop.end(); i++) {
        tot_fitness += 1/(-i->fitness());
    }
    for(std::vector<Robot>::iterator i = subpop.begin(); i < subpop.end(); i++) {
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

void Optimizer::CrossoverStep(subpopulation& subpop) {
    uint num_replaced = subpop.size() * crossover_rate;

    if(crossover == CROSS_NONE) return;

    RobotPair children;
    for(uint i = 0; i < num_replaced; i++) {
        SolutionPair parents;

        parents.first   = &SelectRoulette(subpop);
        parents.second  = &SelectRoulette(subpop);

        parents.first->IncrementAge();
        parents.second->IncrementAge();

        RobotPair p = {*parents.first, *parents.second};

        children = Robot::Crossover(p);
        
        subpop.crossoverFamilyBuffer.push_back({parents, children});
    }
}

void Optimizer::MutateCollect(std::vector<subpopulation>& subpop_list) {
    std::vector<Robot> evalBuf;

    for(subpopulation& subpop : subpop_list) {
        for(AsexualRobotFamily& fam : subpop.mutationFamilyBuffer) {
            evalBuf.push_back(fam.child);
        }
    }
    
    Evaluator::BatchEvaluate(evalBuf);

    uint count = 0;
    for(subpopulation& subpop : subpop_list) {
        for(AsexualRobotFamily& fam : subpop.mutationFamilyBuffer) {
            fam.child = evalBuf[count++];
        }
        Evaluator::pareto_sort(subpop.begin(),subpop.end());
    }

    for(subpopulation& subpop : subpop_list) {
        int max_elite = elitism * subpop.size();

        for(AsexualRobotFamily& fam : subpop.mutationFamilyBuffer) {
            uint r_idx = max_elite + (rand() % (subpop.size()-max_elite));
            Robot* random_robot  = &subpop[r_idx];
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
    }
}

void Optimizer::CrossoverCollect(std::vector<subpopulation>& subpop_list) {
    std::vector<Robot> evalBuf;

    for(subpopulation& subpop : subpop_list) {
        for(SexualRobotFamily& fam : subpop.crossoverFamilyBuffer) {
            evalBuf.push_back(fam.children.first);
            evalBuf.push_back(fam.children.second);
        }
    }

    Evaluator::BatchEvaluate(evalBuf);

    uint count = 0;
    for(subpopulation& subpop : subpop_list) {
        for(SexualRobotFamily& fam : subpop.crossoverFamilyBuffer) {
            fam.children.first = evalBuf[count++];
            fam.children.second = evalBuf[count++];
        }
        Evaluator::pareto_sort(subpop.begin(),subpop.end());
    }

    for(subpopulation& subpop : subpop_list) {
        int max_elite = elitism * subpop.size();
        for(SexualRobotFamily& fam : subpop.crossoverFamilyBuffer) {
            switch(crossover) {
                case CROSS_DC: 
                {
                    float D00, D11, D01, D10;
                    RobotPair P0 = {fam.children.first, *fam.parents.first},
                                  P1 = {fam.children.second, *fam.parents.second},
                                  P2 = {fam.children.first,  *fam.parents.second},
                                  P3 = {fam.children.second, *fam.parents.first };


                    D00 = Robot::Distance(P0);
                    D11 = Robot::Distance(P1);
                    D01 = Robot::Distance(P2);
                    D10 = Robot::Distance(P3);

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
}

void Optimizer::AlpsMutateStep(subpopulation& subpop) {
    for(auto i = subpop.begin(); i < subpop.end(); i++) {
        Robot new_sol;
        switch(mutator){
            case RANDOMIZE:
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
        AsexualRobotFamily fam{i.base(), new_sol};
        subpop.mutationFamilyBuffer.push_back(fam);
    }
}


std::vector<Robot> Optimizer::ALPSSolve() {    
    thread_count = 7;
    niche_count = thread_count;
    subpop_size = pop_size/niche_count;

    float export_thresh[] = {2,3,5,8,13,21,34,10000};
    
    std::vector<std::thread*> threads(thread_count);
    subpop_list = std::vector<subpopulation>(niche_count);
    population = std::vector<Robot>(pop_size);

    if(subpop_size == 0) {
        niche_count = 1;
        subpop_size = pop_size;
        std::vector<Robot>::iterator begin = population.begin();
        std::vector<Robot>::iterator end = population.end();
        subpop_list[0] = subpopulation(begin, end);
        subpop_list[0].export_threshold = 10000;
    } else {
        for(uint i = 0; i < niche_count; i++) {
            std::vector<Robot>::iterator begin = population.begin() + i*subpop_size;
            std::vector<Robot>::iterator end = begin + subpop_size;
            subpop_list[i] = subpopulation(begin, end);
            subpop_list[i].export_threshold = export_thresh[i];
        }
    }

    RandomizePopulation(population);
    std::sort(population.begin(),population.end(),std::greater<Robot>());
    printf("Initial Best: %f\n",population[0].fitness());

    ulong generation = 1;
    std::vector<float>generation_history(population.size());
    for(size_t i = 0; i < population.size(); i++)
        generation_history[i] = population[i].fitness();
    std::vector<float> diversity = updateDiversity(population);
    population_history.push_back({Evaluator::eval_count, generation_history, diversity});

    while(Evaluator::eval_count < max_evals) {
        printf("------MUTATE-----\n");

        for(size_t i = 0; i < subpop_list.size(); i++) {
            MutateStep(subpop_list[i]);
        }

        MutateCollect(subpop_list);

        for(subpopulation& subpop : subpop_list) {
            Evaluator::pareto_sort(subpop.begin(),subpop.end());
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

        for(subpopulation& subpop : subpop_list) {
            Evaluator::pareto_sort(subpop.begin(),subpop.end());
        }

        Evaluator::pareto_sort(population.begin(),population.end());

        diversity = updateDiversity(population);

        Robot archived_solution;
        archived_solution = population[0];
        uint i = 1;
        while (population[i] <= population[i+1] && i < population.size())
        {
            if(archived_solution.fitness() < population[i].fitness())
                archived_solution = population[i];
            i++;
        }

        solution_history.push_back({Evaluator::eval_count, archived_solution});
        fitness_history.push_back({Evaluator::eval_count, archived_solution.fitness()});
        diversity_history.push_back({Evaluator::eval_count, diversity[0]});

        //generation_digits = floor(log10(generation));
        if(generation % print_skip == 0) {
        // if(generation/pow(hist_skip_factor,generation_digits) == 1) {
            for(size_t i = 0; i < population.size(); i++)
                generation_history[i] = population[i].fitness();
            population_history.push_back({Evaluator::eval_count, generation_history, diversity});
        }

        if(generation % print_skip == 0) {
            printf("Generation: %lu, Evlauation: %lu\t%s\n",
                    generation,
                    Evaluator::eval_count,
                    population[0].fitnessReadout().data());
            printf("----PARETO SOLUTIONS----\n");
            uint i = 0;
            Robot sol;
            while(i < pop_size) {
                if(population[i] < population[0]) break;
                sol = population[i];
                printf("%u:\t%s\n",
                    i,
                    population[i].fitnessReadout().data());
                i++;
            }
            printf("----------------------\n");
        }
    }

    Evaluator::pareto_sort(population.begin(),population.end());

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


std::vector<Robot> Optimizer::NoNicheSolve() {
    niche_count = thread_count;
    subpop_size = pop_size/niche_count;
    
    std::vector<std::thread*> threads(niche_count);
    std::vector<Robot> population(pop_size);
    subpop_list = std::vector<subpopulation>(niche_count);
    subpopulation full_pop(population.begin(), population.end());

    if(subpop_size == 0) {
        niche_count = 1;
        subpop_size = pop_size;
        std::vector<Robot>::iterator begin = population.begin();
        std::vector<Robot>::iterator end = population.end();
        subpop_list[0] = subpopulation(begin, end);
    } else {
        for(uint i = 0; i < niche_count; i++) {
            std::vector<Robot>::iterator begin = population.begin() + i*subpop_size;
            std::vector<Robot>::iterator end = begin + subpop_size;
            subpop_list[i] = subpopulation(begin, end);
        }
    }

    RandomizePopulation(population);
    Evaluator::pareto_sort(population.begin(),population.end());
    printf("Initial Best: %f\n",population[0].fitness());

    ulong generation = 1;
    std::vector<float>generation_history(population.size());
    for(size_t i = 0; i < population.size(); i++)
        generation_history[i] = population[i].fitness();
    std::vector<float> diversity = updateDiversity(population);
    population_history.push_back({Evaluator::eval_count, generation_history, diversity});

    while(Evaluator::eval_count < max_evals) {
        printf("------MUTATE-----\n");

        for(size_t i = 0; i < subpop_list.size(); i++) {
            MutateStep(subpop_list[i]);
        }

        MutateCollect(subpop_list);

        Evaluator::pareto_sort(population.begin(),population.end());

        printf("----CROSSOVER----\n");
        CrossoverStep(full_pop);

        for(uint i = 0; i < population.size(); i++) {
            population[i].IncrementAge();
        }

        std::vector<subpopulation> temp_pop_list;
        temp_pop_list.push_back(full_pop);
        CrossoverCollect(temp_pop_list);
        full_pop.crossoverFamilyBuffer.clear();
        
        Evaluator::pareto_sort(population.begin(),population.end());

        diversity = updateDiversity(population);

        Robot archived_solution;
        archived_solution = population[0];
        uint i = 1;
        while (population[i] <= population[i+1] && i < population.size())
        {
            if(archived_solution.fitness() < population[i].fitness())
                archived_solution = population[i];
            i++;
        }

        solution_history.push_back({Evaluator::eval_count, archived_solution});
        fitness_history.push_back({Evaluator::eval_count, archived_solution.fitness()});
        diversity_history.push_back({Evaluator::eval_count, diversity[0]});

        //generation_digits = floor(log10(generation));
        if(generation % print_skip == 0) {
        // if(generation/pow(hist_skip_factor,generation_digits) == 1) {
            for(size_t i = 0; i < population.size(); i++)
                generation_history[i] = population[i].fitness();
            population_history.push_back({Evaluator::eval_count, generation_history, diversity});
        }

        if(generation % print_skip == 0) {
            printf("Generation: %lu, Evlauation: %lu\t%s\n",
                generation,
                Evaluator::eval_count,
                population[0].fitnessReadout().data());
            printf("----PARETO SOLUTIONS----\n");
            uint i = 0;
            Robot sol;
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

    Evaluator::pareto_sort(population.begin(),population.end());

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

std::vector<Robot> Optimizer::Solve() {
    switch(niche){
        case NICHE_NONE:
            return NoNicheSolve();
            break;
        default:
            return NoNicheSolve();
            break;
    }
}
