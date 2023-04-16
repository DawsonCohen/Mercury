#include <chrono>
#include <csignal>
#include <algorithm>
#include <map>
#include <limits>
#include <sstream>
#include <fstream>
#include "NNRobot.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

std::vector<int> NNRobot::hidden_sizes = std::vector<int>{25,25};
uint NNRobot::num_layers = 4;

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

NNRobot::NNRobot(const uint num_masses)
{
    uint hidden_layers = hidden_sizes.size();
    num_layers = hidden_layers+2;

    weights.resize(num_layers - 1);

    weights[0] = Eigen::MatrixXf::Random(hidden_sizes[0], input_size);
    for (uint i = 0; i < hidden_layers-1; i++) {
        weights[i+1] = Eigen::MatrixXf::Random(hidden_sizes[i+1], hidden_sizes[i]);
    }
    
    weights[num_layers-2] = Eigen::MatrixXf::Random(output_size, hidden_sizes[hidden_sizes.size() - 1]);
    
    for(uint i = 0; i < num_masses; i++) {
        float el = (uniform(gen) * M_PI) - M_PI/2;
        float az = uniform(gen) * 2 * M_PI;
        float r = uniform(gen);

        float x = r * sin(el) * cos(az);
        float z = r * sin(el) * sin(az);
        float y = r * cos(el);
        
        Mass m(i,x,y,z);
        addMass(m);
    }

    Build();
}

std::vector<std::pair<uint, float>> get_k_nearest_neighbors(uint i, const std::vector<std::vector<float>>& dists, uint k) {
    std::vector<std::pair<uint, float>> neighbors(dists.size());
    std::vector<uint> kNearest(dists.size());
    for (uint j = 0; j < dists.size(); j++) {
        if(dists[i][j] == 0.0f)
            neighbors[j] = {j, std::numeric_limits<double>::infinity()};
        else
            neighbors[j] = {j, dists[i][j]};
    }
    sort(neighbors.begin(), neighbors.end(), [](const std::pair<uint, float>& a, const std::pair<uint, float>& b) {
        return a.second < b.second;
    });
    
    neighbors.resize(k);
    return neighbors;
}

void NNRobot::Build() {
    springs.clear();
    
    Eigen::MatrixXf input(input_size, masses.size());

    for(uint i = 0; i < masses.size(); i++) {
        Mass m = masses[i];
        // printf("Original Pos: {%f,%f,%f}\n",m.protoPos.x(), m.protoPos.y(), m.protoPos.z());
        input.col(i) << m.protoPos.x(), m.protoPos.y(), m.protoPos.z();
    }

    Eigen::MatrixXf output = forward(input);
    Eigen::MatrixXf positions = output.topRows(3);
    Eigen::MatrixXf material_probs = output.bottomRows(MATERIAL_COUNT);
    
    for(uint i = 0; i < masses.size(); i++) {
        masses[i].pos = masses[i].protoPos = positions.col(i);
        // printf("Pos: {%f,%f,%f}\n",masses[i].pos.x(), masses[i].pos.y(), masses[i].pos.z());
        // printf("New Pos: {%f,%f,%f}\n",masses[i].pos.x(), masses[i].pos.y(), masses[i].pos.z());
        // printf("New Pos: {%f,%f,%f}\n",positions.col(i)[0], positions.col(i)[1], positions.col(i)[2]);

        Eigen::VectorXf mat_prob = material_probs.col(i);
        int maxIdx;
        mat_prob.maxCoeff(&maxIdx);
        masses[i].material = materials::matLookup(maxIdx);
    }

    std::vector<std::vector<float>> dists(masses.size(), std::vector<float>(masses.size()));
    for (uint i = 0; i < masses.size(); i++) {
        for (uint j = i + 1; j < masses.size(); j++) {
            dists[i][j] = dists[j][i] = (masses[i].pos - masses[j].pos).norm();
        }
    }

    uint k = 25;
    for (uint i = 0; i < masses.size(); i++) {
        std::vector<std::pair<uint, float>> neighbors = get_k_nearest_neighbors(i, dists, k);
        for (auto neighbor : neighbors) {
            Material mat1 = masses[i].material;
            Material mat2 = masses[neighbor.first].material;
            std::vector<Material> mats = {mat1, mat2};
            Material mat;
            if(mat1 == materials::air || mat2 == materials::air)
                mat = materials::air;
            else
                mat = Material::avg(mats);
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

    ShiftX(*this);
    ShiftY(*this);
}

void NNRobot::Randomize() {
    for(uint i = 0; i < weights.size(); i++) {
        weights[i] = Eigen::MatrixXf::Random(weights[i].rows(), weights[i].cols());
    }

    Build();
}

void NNRobot::Mutate() {
    int layer = rand() % weights.size();
    int i = rand() % weights[layer].rows();
    int j = rand() % weights[layer].cols();

    weights[layer](i,j) = uniform(gen)*2-1;
}

CandidatePair<NNRobot> NNRobot::Crossover(const CandidatePair<NNRobot>& parents) {
    CandidatePair<NNRobot> children;
    children.first = parents.first;
    children.second = parents.second;
    
    int layer = rand() % parents.first.weights.size();
    int nodeIdx = rand() % parents.first.weights[layer].rows();

    auto row1 = children.first.weights[layer].row(nodeIdx);
    auto row2 = children.second.weights[layer].row(nodeIdx);

    row1.swap(row2);

    return children;
}

float NNRobot::Distance(const CandidatePair<NNRobot>& robots) {
    float distance = 0;
    for(uint i = 0; i < NNRobot::num_layers - 1; i++) {
        auto W1 = robots.first.weights[i];
        auto W2 = robots.second.weights[i];
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
    std::ifstream file(filename);
    if (file)
    {
        std::string line;
        std::stringstream ss;
        Eigen::MatrixXf matrix;
        int current_row = 0;
        while (std::getline(file, line))
        {
            if (line.empty())
            {
                if (matrix.size() > 0)
                    weights.push_back(matrix);
                matrix = Eigen::MatrixXf();
                current_row = 0;
            }
            else
            {
                ss.clear();
                ss.str(line);
                int current_col = 0;
                float value;
                while (ss >> value)
                {
                    if (current_row == 0 && current_col == 0)
                    {
                        int rows = std::count(line.begin(), line.end(), ',') + 1;
                        int cols = std::count(ss.str().begin(), ss.str().end(), ',') + 1;
                        matrix.resize(rows, cols);
                    }
                    matrix(current_row, current_col) = value;
                    ++current_col;
                    if (ss.peek() == ',')
                        ss.ignore();
                }
                ++current_row;
            }
        }
        if (matrix.size() > 0)
            weights.push_back(matrix);
        file.close();
    }
}
