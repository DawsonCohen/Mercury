#include <chrono>
#include <csignal>
#include <algorithm>
#include <map>
#include <limits>
#include <sstream>
#include <fstream>
#include "NNRobot.h"
#include "knn.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

#define EPS 1e-3


std::vector<unsigned int> NNRobot::hidden_sizes = std::vector<unsigned int>{25,25};

std::vector<Mass> NNRobot::randMasses;
bool NNRobot::randMassesFilled = false;

unsigned int NNRobot::num_layers = 4;
int NNRobot::crossover_neuron_count = 5;
int NNRobot::mutation_weight_count = 10;
int NNRobot::springs_per_mass = 25;
unsigned int NNRobot::maxMasses = 1728;
unsigned int NNRobot::maxSprings = NNRobot::maxMasses * NNRobot::springs_per_mass;

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

    Build();
}

void NNRobot::Randomize() {
    for(unsigned int i = 0; i < weights.size(); i++) {
        weights[i] = Eigen::MatrixXf::Random(weights[i].rows(), weights[i].cols());
    }

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
    
    for(int i = 0; i < crossover_neuron_count; i++) {
        int layer = rand() % parents.first.weights.size();
        int nodeIdx = rand() % parents.first.weights[layer].rows();

        auto row1 = children.first.weights[layer].row(nodeIdx);
        auto row2 = children.second.weights[layer].row(nodeIdx);

        row1.swap(row2);
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
    
    // TODO
    for(size_t i = 0; i < pop_size; i++) {
        for(size_t j  = 0; j < pop_size; j++){
            CandidatePair<NNRobot> pair = {pop[i], pop[j]};
            diversity[i] += NNRobot::Distance(pair);
        }
    }
    return diversity;
}

std::string NNRobot::Encode() const {
    std::stringstream ss;
    ss << "type=NNRobot\n";
    ss << "masses=";
    for(unsigned int i = 0; i < masses.size(); i++) {
        ss << masses[i];
        ss << (i < (masses.size()- 1) ? ";" : "\n");
    }
    ss << "springs=";
    for(unsigned int i = 0; i < springs.size(); i++) {
        ss << springs[i];
        ss << (i < springs.size()- 1 ? ";" : "\n");
    }
    return ss.str();
}

void NNRobot::Decode(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error decoding robot file: " << filename << std::endl;
        exit(1);
    }
    
    std::string line;
    masses.clear();
    springs.clear();

    std::getline(infile, line);
    std::getline(infile, line);
    std::size_t pos = line.find('=');
    std::string key = line.substr(0, pos);
    std::string value = line.substr(pos+1);
    if(key == "masses") {
        std::istringstream lineStream(value);
        std::string cell;

        while (std::getline(lineStream, cell, ';')) {
            std::istringstream mass(cell);
            std::string param;
            uint id;
            float x,y,z, m;
            Material mat;

            std::getline(mass, param, ',');
            id = std::stoi(param);
            std::getline(mass, param, ',');
            x = std::stof(param);
            std::getline(mass, param, ',');
            y = std::stof(param);
            std::getline(mass, param, ',');
            z = std::stof(param);
            std::getline(mass, param, ',');
            m = std::stof(param);
            std::getline(mass, param, ',');
            mat = materials::decode(stoi(param));
            masses.push_back(Mass(id, x, y, z, m, mat));
        }
    }
    std::getline(infile, line);
    pos = line.find('=');
    key = line.substr(0, pos);
    value = line.substr(pos+1);
    if(key == "springs") {
        std::istringstream lineStream(value);
        std::string cell;

        while (std::getline(lineStream, cell, ';')) {
            std::istringstream spring(cell);
            std::string param;
            uint16_t m0, m1;
            float rl, ml;
            Material mat;

            std::getline(spring, param, ',');
            m0 = std::stoi(param);
            std::getline(spring, param, ',');
            m1 = std::stoi(param);
            std::getline(spring, param, ',');
            rl = std::stof(param);
            std::getline(spring, param, ',');
            ml = std::stof(param);
            std::getline(spring, param, ',');
            mat = materials::decode(std::stoi(param));
            Spring s = {m0, m1, rl, ml, mat};
            springs.push_back(s);
        }
    }
    updateBaseline();
}

void NNRobot::Build() {
    springs.clear();

    // auto start = std::chrono::high_resolution_clock::now();
    forward();

    std::vector<std::vector<float>> dists(masses.size(), std::vector<float>(masses.size()));
    for (size_t i = 0; i < masses.size(); i++) {
        for (size_t j = i + 1; j < masses.size(); j++) {
            dists[i][j] = dists[j][i] = (masses[i].pos - masses[j].pos).norm();
        }
    }
    // auto end = std::chrono::high_resolution_clock::now();
    // auto execute_time = std::chrono::duration<float>(end - start).count();
    // printf("INFERENCE IN %f SECONDS\n", execute_time);

    // start = std::chrono::high_resolution_clock::now();
    
    auto knns = KNN::KNN(*this, springs_per_mass);
    
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