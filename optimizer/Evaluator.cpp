#include "Evaluator.h"
#include <iterator>


ulong Evaluator::eval_count = 0;
float Evaluator::baselineTime = 5.0f;
float Evaluator::evaluationTime = 10.0f;
Simulator Evaluator::Sim = Simulator();

void Evaluator::Initialize(uint pop_size, float base_time, float eval_time) {
    Robot prototype;
    
    Sim.Initialize(prototype, pop_size*1.5);
    baselineTime = base_time;
    evaluationTime = eval_time;
}

void Evaluator::BatchEvaluate(std::vector<Robot>& robots) {
    printf("EVALUATING %lu ROBOTS\n",robots.size());

    std::vector<Element> robot_elements;
    for(uint i = 0; i < robots.size(); i++) {
        Robot R = robots[i];
        robot_elements.push_back({R.getMasses(), R.getSprings()});
    }

    Sim.setMaxTime(baselineTime);

    std::vector<ElementTracker> trackers = Sim.Simulate(robot_elements);
    
    std::vector<Element> results = Sim.Collect(trackers);

    for(uint i = 0; i < robots.size(); i++) {
        robots[i].Update(results[i]);
    }

    for(Robot& r : robots) {
        eval_count++;
        r.mBaseCOM = Robot::calcMeanPos(r);
    }

    Sim.setMaxTime(evaluationTime-baselineTime);

    // TODO: "continue" simulation without copying vector again
    trackers = Sim.Simulate(robot_elements);
    results = Sim.Collect(trackers);

    for(uint i = 0; i < robots.size(); i++) {
        robots[i].Update(results[i]);
    }

    for(Robot& r : robots) {
        eval_count++;
        Robot::calcFitness(r);
        
        r.Reset();
    }


    Sim.Reset();
}

float Evaluator::Distance(const RobotPair& robots) {
    float dist = 0;
    std::vector<Spring> s1 = robots.first.getSprings();
    std::vector<Spring> s2 = robots.second.getSprings();
    for(size_t i = 0; i < s1.size(); i++) {
        dist += !(s1[i].material == s2[i].material);
    }
    return dist;
}