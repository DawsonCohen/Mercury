#include <chrono>
#include <csignal>
#include <algorithm>
#include <map>
#include "NNRobot.h"
#include "knn.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

#define EPS 1e-3


std::vector<unsigned int> NNRobot::hidden_sizes = std::vector<unsigned int>{25,25};

std::vector<Mass> NNRobot::randMasses;
bool NNRobot::randMassesFilled = false;

unsigned int NNRobot::num_layers = 4;
float NNRobot::crossover_neuron_count = .2;
int NNRobot::mutation_weight_count = 10;
int NNRobot::springs_per_mass = 25;
unsigned int NNRobot::maxMasses = 1728;
unsigned int NNRobot::maxSprings = NNRobot::maxMasses * NNRobot::springs_per_mass;
CrossoverDistribution NNRobot::crossover_distribution = CROSS_DIST_BINOMIAL;
CrossoverType NNRobot::crossover_type = CROSS_CONTIGUOUS;

void ShiftY(NNRobot& R) {
    bool setFlag = false;
    float minY = -1;
    for(const Mass& m : R.getMasses()) {
        if(!m.active) continue;
        if(!setFlag || m.protoPos.y() < minY) {
            setFlag = true;
            minY = m.protoPos.y();
        }
    }

    Eigen::Vector3f translation(0.0f,-minY,0.0f);
    R.translate(translation);
}

void ShiftX(NNRobot& R) {
    bool setFlag = false;
    float maxX = 0;
    for(const Mass& m : R.getMasses()) {
        if(!m.active) continue;
        if(!setFlag || m.protoPos.x() > maxX) {
            setFlag = true;
            maxX = m.protoPos.x();
        }
    }

    Eigen::Vector3f translation(-maxX,0.0f,0.0f);
    R.translate(translation);
}

NNRobot::NNRobot()
{
    unsigned int hidden_layers = hidden_sizes.size();
    num_layers = hidden_layers+2;

    weights.resize(num_layers - 1);

    weights[0] = Eigen::MatrixXf::Random(hidden_sizes[0], input_size);
    for (unsigned int i = 0; i < hidden_layers-1; i++) {
        weights[i+1] = Eigen::MatrixXf::Random(hidden_sizes[i+1], hidden_sizes[i]);
    }
    
    weights[num_layers-2] = Eigen::MatrixXf::Random(output_size, hidden_sizes[hidden_sizes.size() - 1]);

    // Build();
}

void NNRobot::Randomize() {
    for(unsigned int i = 0; i < weights.size(); i++) {
        weights[i] = Eigen::MatrixXf::Random(weights[i].rows(), weights[i].cols());
    }

    mAge = 0;

    Build();
}

void NNRobot::Mutate() {
    for(int i = 0; i < mutation_weight_count; i++) {
        int layer = rand() % weights.size();
        int row = rand() % weights[layer].rows();
        int col = rand() % weights[layer].cols();

        weights[layer](row,col) = uniform(gen)*2-1;
    }

    Build();
}

CandidatePair<NNRobot> NNRobot::Crossover(const CandidatePair<NNRobot>& parents) {
    CandidatePair<NNRobot> children;
    int crossover_count, layer;

    switch(crossover_type)
    { 
        case CROSS_INDIVIDUAL:
        {
            layer = 0;
        }
        break;
        case CROSS_CONTIGUOUS:
        {
            layer = rand() % parents.first.weights.size();
        }
        break;
    }

    crossover_count = crossover_neuron_count*parents.first.weights[layer].rows();

    if (crossover_distribution == CROSS_DIST_BINOMIAL) {
        std::normal_distribution<> cross_dist(crossover_count, crossover_count/3.0);
        crossover_count = (int)cross_dist(gen);
    }

    switch(crossover_type)
    { 
        case CROSS_INDIVIDUAL:
        {
            for(int i = 0; i < crossover_count; i++) {
                layer = rand() % parents.first.weights.size();
                int nodeIdx = rand() % parents.first.weights[layer].rows();

                auto row1 = children.first.weights[layer].row(nodeIdx);
                auto row2 = children.second.weights[layer].row(nodeIdx);

                row1.swap(row2);
            }
        }
        break;
        case CROSS_CONTIGUOUS:
        {
            int nodeIdx;
            if(crossover_count >= parents.first.weights[layer].rows()) {
                crossover_count = parents.first.weights[layer].rows();
                nodeIdx = 0;
            } else {
                nodeIdx = rand() % (parents.first.weights[layer].rows()-crossover_count);
            }

            for(int i = nodeIdx; i < crossover_count+nodeIdx; i++) {

                auto row1 = children.first.weights[layer].row(i);
                auto row2 = children.second.weights[layer].row(i);

                row1.swap(row2);
            }
        }
        break;
    }

    children.first.Build();
    children.second.Build();

    uint maxAge = parents.first.mAge;
    if(parents.second.mAge > maxAge)
        maxAge = parents.second.mAge;

    children.first.mAge = maxAge+1;
    children.second.mAge = maxAge+1;
    
    return children;
}

float NNRobot::Distance(const CandidatePair<NNRobot>& candidates) {
    float distance = 0;
    for(unsigned int i = 0; i < NNRobot::num_layers - 1; i++) {
        auto W1 = candidates.first.weights[i];
        auto W2 = candidates.second.weights[i];
        distance += (W1-W2).norm();
    }
    return distance;
}

std::vector<float> NNRobot::findDiversity(std::vector<NNRobot> pop) {
    size_t pop_size = pop.size();
    std::vector<float> diversity(pop_size, 0);
    
    // // TODO
    // for(size_t i = 0; i < pop_size; i++) {
    //     for(size_t j  = 0; j < pop_size; j++){
    //         CandidatePair<NNRobot> pair = {pop[i], pop[j]};
    //         diversity[i] += NNRobot::Distance(pair);
    //     }
    // }
    return diversity;
}

void NNRobot::Build() {
    springs.clear();

    // auto start = std::chrono::high_resolution_clock::now();
    forward();

    // auto end = std::chrono::high_resolution_clock::now();
    // auto execute_time = std::chrono::duration<float>(end - start).count();
    // printf("INFERENCE IN %f SECONDS\n", execute_time);

    // start = std::chrono::high_resolution_clock::now();
    
    auto knns = KNN::KNN(this->masses, springs_per_mass);
    
    // end = std::chrono::high_resolution_clock::now();
    // execute_time = std::chrono::duration<float>(end - start).count();
    // printf("KNN IN %f SECONDS\n", execute_time);

    // start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < masses.size(); i++) {
        auto neighbors = knns[i];
        Material mat1 = masses[i].material,
                 mat2, mat;

        for (auto neighbor : neighbors) {
            if(neighbor.second < EPS) {
                mat = materials::air;
            } else {
                mat2 = masses[neighbor.first].material;
                mat = materials::decode(mat1.encoding | mat2.encoding);
            }

            Spring s = {(uint16_t)i, neighbor.first, neighbor.second, neighbor.second, mat};
            springs.push_back(s);
        }
    }

    for(Spring& s : springs) {
        if(s.material != materials::air) {
            masses[s.m0].active = true;
            masses[s.m1].active = true;
        }
    }

    // end = std::chrono::high_resolution_clock::now();
    // execute_time = std::chrono::duration<float>(end - start).count();
    // printf("SPRINGS CREATED IN %f SECONDS\n", execute_time);

    ShiftX(*this);
    ShiftY(*this);

    updateBaseline();
}