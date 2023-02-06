#ifndef __Optimizer_H__
#define __Optimizer_H__

#include <algorithm>
#include <chrono>
#include <iterator>
#include <mutex>
#include <queue>
#include <random>
#include <map>
#include <vector>
#include "Evaluator.h"
#include "robot.h"

struct subpopulation {
    float export_threshold = 0;
    std::vector<Robot>::iterator mBegin{};
    std::vector<Robot>::iterator mEnd{};
    size_t mSize = 0;
    std::mutex a_mutex;
    std::mutex p_mutex;
    std::queue<Robot> admissionBuffer;
    std::vector<AsexualRobotFamily> mutationFamilyBuffer;
    std::vector<SexualRobotFamily> crossoverFamilyBuffer;

    subpopulation(void) {
    }

    subpopulation(const subpopulation& src): 
    export_threshold(src.export_threshold), mBegin(src.mBegin),
    mEnd(src.mEnd), mSize(src.mSize),
    admissionBuffer(src.admissionBuffer),
    mutationFamilyBuffer(src.mutationFamilyBuffer),
    crossoverFamilyBuffer(src.crossoverFamilyBuffer)
    {}

    subpopulation(std::vector<Robot>::iterator _begin,
                    std::vector<Robot>::iterator _end,
                    int _et = 0) {
        export_threshold = _et;

        mBegin = _begin;
        mEnd = _end;
        mSize = (size_t) (_end-_begin);
    }

    std::vector<Robot>::iterator begin() {
        return mBegin;
    }

    std::vector<Robot>::iterator end() {
        return mEnd;
    }

    size_t size() { return mSize; }

    subpopulation &operator=(const subpopulation r) {
      export_threshold=r.export_threshold;
      mBegin=r.mBegin;
      mEnd=r.mEnd;
      mSize = r.mSize;
      return *this;
   }

    Robot& operator[](size_t i) {
        return *(mBegin+i);
    }
};

class Optimizer {
public:
    enum MutationStrat {
        RANDOMIZE = 0,
        MUTATE = 1,
    };

    // TODO
    enum MutationRate {
        MUTATE_CONST = 0,
        MUTATE_SIM_ANNEALING = 1
    };

    enum CrossoverStrat {
        CROSS_NONE = -1,
        CROSS_BEAM = 0,
        CROSS_SWAP = 1,
        CROSS_DC = 2
    };

    enum NichingStrat {
        NICHE_NONE = 0,
        NICHE_HFC = 1,
        NICHE_MALPS = 2
    };
    
    ulong max_evals = 1e4;
    
    uint pop_size = 100;
    uint niche_count = 1;
    uint thread_count = 12;
    uint steps_to_combine = 1;
    uint calibration_steps = (uint) 5;
    uint exchange_steps = 5e3;

    uint max_heap = 1024;

    uint hist_skip_factor = 10;
    uint print_skip = 1;

    // TODO
    float mutation_rate = 0.6;
    float crossover_rate = 0.7;
    float elitism = 0.1;
    bool prune = false;
    bool snip = false;

    MutationStrat mutator = MUTATE;
    CrossoverStrat crossover = CROSS_DC;
    NichingStrat niche = NICHE_MALPS;

    unsigned seed;
    std::default_random_engine gen;
    std::uniform_real_distribution<> uniform;
    std::gamma_distribution<> gamma;

    Optimizer();
    ~Optimizer(void);

private:
    float P,p;

    ulong eval_count = 0;
    ulong subpop_size;
    
    std::vector<Robot> population;
    std::vector<subpopulation> subpop_list;

    std::vector<std::tuple<ulong,Robot>> solution_history;
    std::vector<std::tuple<ulong,float>> fitness_history;
    std::vector<std::tuple<ulong,float>> diversity_history;
    std::vector<std::tuple<ulong,std::vector<float>,std::vector<float>>> population_history;
    std::vector<Robot> solutions;
    
    void RandomizePopulation(std::vector<Robot>& population);
    Robot RandomizeSolution(Robot&);
    Robot MutateSolution(Robot&); // TODO Rename
    void SimulatedAnnealingStep(Robot&);
    // void MutateStep(std::vector<Robot>&);
    void MutateStep(subpopulation& subpop);
    void MutateCollect(std::vector<subpopulation>& subpop_list);
    
    void AlpsMutateStep(subpopulation& subpop);
    void AlpsCollectStep(std::vector<subpopulation>& subpop_list);

    void CrossoverStep(subpopulation&);
    void CrossoverCollect(std::vector<subpopulation>& subpop_list);
    void CalibrateStep(void);
    void Calibrate(void);
    void HFCStep(subpopulation&, uint);
    void SteadyStateReplace(subpopulation& subpop, RobotPair& parents, RobotPair& children, size_t P1, size_t P2);
    void SteadyStateMutate(Robot&);
    void SteadyStateStep(subpopulation&);
    void MALPSCrossover(std::vector<Robot>&, uint);
    void MALPSStep(std::vector<Robot>&, uint);

    std::vector<Robot> NoNicheSolve();
    std::vector<Robot> HFCSolve();
    std::vector<Robot> ALPSSolve();
    std::vector<Robot> MALPSSolve();

public:
    void reset(void) {
        Evaluator::eval_count = 0;
        solution_history.clear();
        fitness_history.clear();
        population_history.clear();
    }
    std::vector<Robot> Solve();
    
    std::vector<Robot>& getSolutions() {return solutions;}
    
    std::vector<std::tuple<ulong, Robot>>& getSolutionHistory() {return solution_history;}
    std::vector<std::tuple<ulong, float>>& getFitnessHistory() {return fitness_history;}
    std::vector<std::tuple<ulong,std::vector<float>,std::vector<float>>>& getPopulationHistory() {return population_history;}
};

#endif