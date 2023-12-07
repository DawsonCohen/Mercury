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
#include "EvoDevo/Optimizer/Evaluator.h"
#include "EvoDevo/Util/util.h"

namespace EvoDevo {

    template<typename T>
    using Solution = T*;

    template<typename T>
    struct SolutionPair {
        Solution<T> first;
        Solution<T> second;
    };

    template<typename T>
    struct SexualFamily {
        SolutionPair<T> parents;
        CandidatePair<T> children;
    };

    template<typename T>
    struct AsexualFamily {
        Solution<T> parent;
        T child;
    };

    template<typename T>
    struct subpopulation {
        float export_threshold = 0;

        typename std::vector<T>::iterator mBegin{};
        typename std::vector<T>::iterator mEnd{};

        size_t mSize = 0;
        std::mutex a_mutex;
        std::mutex p_mutex;
        std::queue<T> admissionBuffer;
        std::vector<AsexualFamily<T>> mutationFamilyBuffer;
        std::vector<SexualFamily<T>> crossoverFamilyBuffer;

        subpopulation(void) {
        }

        subpopulation(const subpopulation<T>& src): 
        export_threshold(src.export_threshold), mBegin(src.mBegin),
        mEnd(src.mEnd), mSize(src.mSize),
        admissionBuffer(src.admissionBuffer),
        mutationFamilyBuffer(src.mutationFamilyBuffer),
        crossoverFamilyBuffer(src.crossoverFamilyBuffer)
        {}

        subpopulation(typename std::vector<T>::iterator _begin,
                        typename std::vector<T>::iterator _end,
                        int _et = 0) {
            export_threshold = _et;

            mBegin = _begin;
            mEnd = _end;
            mSize = (size_t) (_end-_begin);
            std::vector<uint> pFlags(mSize, 0);
        }

        typename std::vector<T>::iterator begin() {
            return mBegin;
        }

        typename std::vector<T>::iterator end() {
            return mEnd;
        }

        size_t size() { return mSize; }

        subpopulation &operator=(const subpopulation<T> r) {
        export_threshold=r.export_threshold;
        mBegin=r.mBegin;
        mEnd=r.mEnd;
        mSize = r.mSize;
        return *this;
    }

        T& operator[](size_t i) {
            return *(mBegin+i);
        }
    };

    template<typename T>
    class Optimizer {
        
    public:
        Optimizer();
        ~Optimizer(void);

        void reset(void) {
            Evaluator<T>::eval_count = 0;
        }
        std::vector<T> Solve(Config config = Config());
        std::vector<T>& getSolutions() {return solutions;}

    private:
        void RandomizePopulation(std::vector<T>& population);
        void RandomizeSolution(Solution<T>);
        void MutateSolution(Solution<T>);
        void SimulatedAnnealingStep(T&);
        void ChildStep(subpopulation<T>& subpop);

        void CalibrateStep(void);
        void Calibrate(void);
        void SteadyStateReplace(subpopulation<T>& subpop, CandidatePair<T>& parents, CandidatePair<T>& children, size_t P1, size_t P2);
        void SteadyStateMutate(T&);
        void SteadyStateStep(subpopulation<T>&);

        void WriteSolutions(const std::vector<T>& solutions, const std::string& directory);

        std::vector<T> NoNicheSolve();
        
    private:
        ulong max_evals = 1e4;
        ulong eval_count = 0;
        
        uint pop_size = 100;
        uint niche_count = 1;
        uint thread_count = 12;
        uint steps_to_combine = 1;
        uint calibration_steps = (uint) 5;
        uint exchange_steps = 5e3;

        uint max_heap = 1024;

        uint hist_skip_factor = 10;
        uint print_skip = 1;

        float mutation_crossover_threshold = 0.5;
        float child_pop_size = .8;
        uint injection_rate = 1;

        float elitism = 0.1;

        MutationStrat mutator = MUTATE;
        CrossoverStrat crossover = CROSS_DC;
        ReplacementStrat replacement = PARETO;
        NichingStrat niche = NICHE_ALPS;

        unsigned seed;
        std::default_random_engine gen;
        std::uniform_real_distribution<> uniform_real;
        std::uniform_int_distribution<> uniform_int;
        std::gamma_distribution<> gamma;

        std::string working_directory;
        FileWriter fitness_history_writer;
        FileWriter diversity_history_writer;
        FileWriter population_history_writer;

        ulong subpop_size;
        
        std::vector<T> population;
        std::vector<subpopulation<T>> subpop_list;

        std::vector<T> solutions;
        std::vector<T> pareto_solutions;

        Config config;

    };
}

#include "optimizer_impl.hpp"

#endif