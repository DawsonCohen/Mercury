#include "Evaluator.h"
#include <iterator>


ulong Evaluator::eval_count = 0;
Simulator Evaluator::Sim = Simulator();

void Evaluator::Initialize(uint pop_size, float max_time) {
    Robot prototype;
    
    Sim.Initialize((Element) prototype, pop_size*3);
    Sim.setMaxTime(max_time);
}

void Evaluator::BatchEvaluate(std::vector<Robot>& robots) {
    printf("EVALUATING %lu ROBOTS\n",robots.size());

    std::vector<Element> robot_elements(robots.size());
    for(uint i = 0; i < robots.size(); i++) {
        robot_elements[i] = robots[i];
    }


    std::vector<ElementTracker> trackers = Sim.Simulate(robot_elements);
    
    std::vector<Element> results = Sim.Collect(trackers);

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