#include <chrono>
#include <csignal>
#include <algorithm>
#include <map>
#include <limits>
#include "NNRobot.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

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

NNRobot::NNRobot(const uint num_masses, const std::vector<int>& hidden_sizes)
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
    printf("RANDOMIZING\n");
    for(uint i = 0; i < weights.size(); i++) {
        weights[i] = Eigen::MatrixXf::Random(weights[i].rows(), weights[i].cols());
    }

    Build();
}
