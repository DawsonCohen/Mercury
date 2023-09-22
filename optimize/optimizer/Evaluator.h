#ifndef __EVALUATOR_H__
#define __EVALUATOR_H__

#include "candidate.h"
#include "Simulator.h"
#include "config.h"
#include <vector>
#include <algorithm>

template<typename T>
class Evaluator {
public:
    static ulong eval_count;
    static Simulator Sim;
    static float baselineTime;
    static float evaluationTime;
    static Config::Simulator sim_config;

    static T protoRobot;

    static void Initialize(Config config);
    static void BatchEvaluate(std::vector<T>&);

    static void pareto_classify(typename std::vector<T>::iterator begin, typename std::vector<T>::iterator end) {
        for(auto i = begin; i < end; i++) {
            i->mParetoLayer = 0;
        }

        int Delta;
        uint run = 0;
        while(true) {
            Delta = 0;
            for(auto i = begin; i < end; i++) {
                if(i->mParetoLayer < run) continue;
                for(auto j = begin; j < end; j++) {
                    if(j->mParetoLayer < run || i == j) continue;
                    if(j->dominates(*i)) {
                        i->mParetoLayer++;
                        Delta++;
                        break;
                    }
                }
            }
            if(Delta == 0) break;
            run++;
        }
    }

    static void pareto_sort(typename std::vector<T>::iterator begin, typename std::vector<T>::iterator end) {
        pareto_classify(begin, end);
        std::sort(begin,end,[](const T a, const T b) {
            if(a > b)
                return true;
            else if(a.paretoLayer() == b.paretoLayer())
                return a.fitness() > b.fitness();
            
            return false;
        });
    }
};

#include "Evaluator_impl.h"

#endif