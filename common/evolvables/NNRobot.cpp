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

#define EPS 0.0001

std::vector<int> NNRobot::hidden_sizes = std::vector<int>{25,25};
unsigned int NNRobot::num_layers = 4;
int NNRobot::crossover_neuron_count = 5;
int NNRobot::mutation_weight_count = 10;
int NNRobot::springs_per_mass = 25;

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

NNRobot::NNRobot(const unsigned int num_masses) :
    maxMasses(num_masses), maxSprings(num_masses*25)
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
    ss << "architecture=";
    for (unsigned int i = 0; i < hidden_sizes.size(); i++) {
        ss << hidden_sizes[i];
        ss << (i < hidden_sizes.size()- 1 ? "," : "\n");
    }
    ss << "fitness=" << mFitness << std::endl;
    for (const auto& matrix : weights)
    {
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                
                ss << matrix(i, j);
                if (j != matrix.cols() - 1)
                    ss << ",";
            }
            ss << "\n";
        }
        ss << "\n";
    }
    return ss.str();
}

void NNRobot::Decode(const std::string& filename) {
    std::ifstream infile(filename);
    if (infile)
    {
        std::string line;
        Eigen::MatrixXf matrix;

        std::vector<std::vector<float>> currentMatrixData;

        weights.clear();
        hidden_sizes.clear();
        std::getline(infile, line);
        std::getline(infile, line);

        // Split line into key-value pair
        std::size_t pos = line.find('=');
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos+1);
        if(key == "architecture") {
            std::istringstream lineStream(value);
            std::string cell;

            while (std::getline(lineStream, cell, ',')) {
                hidden_sizes.push_back(std::stoi(cell));
            }
        }
        num_layers = hidden_sizes.size() + 2;

        std::getline(infile, line);

        // Split line into key-value pair
        pos = line.find('=');
        key = line.substr(0, pos);
        value = line.substr(pos+1);
        if(key == "fitness") {
            mFitness = stof(value);
        }

        while (std::getline(infile, line)) {
            if (line.empty()) {
                // Blank line indicates the end of the current matrix.
                if (!currentMatrixData.empty()) {
                    int numRows = currentMatrixData.size();
                    int numCols = (numRows > 0) ? currentMatrixData[0].size() : 0;
                    Eigen::MatrixXf eigenMatrix(numRows, numCols);

                    // Populate the Eigen matrix with the current matrix data.
                    for (int i = 0; i < numRows; ++i) {
                        for (int j = 0; j < numCols; ++j) {
                            eigenMatrix(i, j) = currentMatrixData[i][j];
                        }
                    }

                    weights.push_back(eigenMatrix);
                    currentMatrixData.clear();
                }
            } else {
                // Split the line into values and add to the current matrix data.
                std::vector<float> row;
                std::istringstream lineStream(line);
                std::string cell;

                while (std::getline(lineStream, cell, ',')) {
                    row.push_back(std::stod(cell));
                }

                currentMatrixData.push_back(row);
            }
        }
        infile.close();
    }

    Build();
}

void NNRobot::Build() {
    springs.clear();

    // auto start = std::chrono::high_resolution_clock::now();
    forward();

    std::vector<std::vector<float>> dists(masses.size(), std::vector<float>(masses.size()));
    for(unsigned int i = 0; i < masses.size(); i++) {
        masses[i].pos = masses[i].pos + 0.003f*(Eigen::Vector3f::Random(3));
    }
    for (unsigned int i = 0; i < masses.size(); i++) {
        for (unsigned int j = i + 1; j < masses.size(); j++) {
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
    for (unsigned int i = 0; i < masses.size(); i++) {
        auto neighbors = knns[i];
        Material mat1 = masses[i].material;

        for (auto neighbor : neighbors) {
            if(neighbor.second < EPS) mValid = false;

            Material mat2 = masses[neighbor.first].material;
            Material mat = materials::getCompositeMaterials(mat1.encoding | mat2.encoding);

            Spring s = {i, neighbor.first, neighbor.second, neighbor.second, mat};
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