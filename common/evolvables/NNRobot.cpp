#include <chrono>
#include <csignal>
#include <algorithm>
#include <map>
#include <thread>
#include "NNRobot.h"
#include "triangulation.h"

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
unsigned int NNRobot::maxMasses = 1708;
unsigned int NNRobot::maxSprings = NNRobot::maxMasses * NNRobot::springs_per_mass;
CrossoverDistribution NNRobot::crossover_distribution = CROSS_DIST_BINOMIAL;
CrossoverType NNRobot::crossover_type = CROSS_CONTIGUOUS;

void ShiftY(NNRobot& R) {
    bool setFlag = false;
    float minY = -1;
    for(const Mass& m : R.getMasses()) {
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
}

void NNRobot::Randomize() {
    for(unsigned int i = 0; i < weights.size(); i++) {
        weights[i] = Eigen::MatrixXf::Random(weights[i].rows(), weights[i].cols());
    }

    mAge = 0;
    // Build();
}

void NNRobot::Mutate() {
    for(int i = 0; i < mutation_weight_count; i++) {
        int layer = rand() % weights.size();
        int row = rand() % weights[layer].rows();
        int col = rand() % weights[layer].cols();

        weights[layer](row,col) = uniform(gen)*2-1;
    }
    // Build();
}

CandidatePair<NNRobot> NNRobot::Crossover(const CandidatePair<NNRobot>& parents) {
    CandidatePair<NNRobot> children;
    int crossover_count, layer = 0;

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

    // children.first.Build();
    // children.second.Build();

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

void BuildSubset(std::vector<NNRobot>& robots, size_t begin, size_t end) {
    for(uint i = begin; i < end; i++) {
        robots[i].Build();
    }
}

void NNRobot::BatchBuild(std::vector<NNRobot>& robots) {
    const auto processor_count = std::thread::hardware_concurrency() - 1;
    unsigned int active_threads = min(robots.size(), processor_count);
    unsigned int robots_per_thread = robots.size() / active_threads;
    
    fillRandMasses(maxMasses);
    std::vector<std::thread> threads;
    uint begin, end;
    for(unsigned int i = 0; i < active_threads; i++) {
        begin = i*robots_per_thread;
        end = min((i+1)*robots_per_thread, robots.size());
        threads.emplace_back(BuildSubset, std::ref(robots), begin, end);
    }

    for (auto& thread : threads) {
        thread.join();
    }
}

/*
 * 1) Sorts masses such that boundary masses are a continguous 
 * 2) Updaes mass id's 
 * Input :  an equal-sized vector of masses and boundary flags
 * Output:  a map from the original index to the new index
*/
void sortBoundaryMasses(std::vector<Mass>& masses, Triangulation::Mesh& mesh) {
    std::vector<uint16_t> idxMap(masses.size());

    // Can push to batched GPU sort if need be
    std::sort(masses.begin(), masses.end(),[mesh](const Mass a, const Mass b) {
            return mesh.isBoundaryVertexFlags[a.id] < mesh.isBoundaryVertexFlags[b.id];
        });
    for(uint i = 0; i < masses.size(); i++) {
        idxMap[masses[i].id] = i;
        masses[i].id = i;
    }
    for(auto& e : mesh.edges) {
        e.v1 = idxMap[e.v1];
        e.v2 = idxMap[e.v2];
    }
    for(auto& f : mesh.facets) {
        f.v1 = idxMap[f.v1];
        f.v2 = idxMap[f.v2];
        f.v3 = idxMap[f.v3];
    }
}

void NNRobot::Build() {
    springs.clear();

    // auto start = std::chrono::high_resolution_clock::now();
    forward();

    // auto end = std::chrono::high_resolution_clock::now();
    // auto execute_time = std::chrono::duration<float>(end - start).count();
    // printf("INFERENCE IN %f SECONDS\n", execute_time);

    // start = std::chrono::high_resolution_clock::now();
    auto triangulation = Triangulation::AlphaShape(this->masses);

    // auto triangulation = Triangulation::KNN(this->masses,springs_per_mass);
    
    // end = std::chrono::high_resolution_clock::now();
    // execute_time = std::chrono::duration<float>(end - start).count();
    // printf("KNN IN %f SECONDS\n", execute_time);

    // start = std::chrono::high_resolution_clock::now();

    sortBoundaryMasses(masses, triangulation);
    for (auto facet : triangulation.facets) {
        uint16_t m1 = facet.v1,
                 m2 = facet.v2,
                 m3 = facet.v3;
        Face f = {m1, m2, m3};
        faces.push_back(f);
    }

    for (auto edge : triangulation.edges) {
        uint16_t m1 = edge.v1,
                 m2 = edge.v2;
        float dist = edge.dist;
        Material mat1, mat2, mat;

        // std::cout << m1 << ", " << m2 << ", " << dist << std::endl;

        if(dist < EPS) {
            // mat = materials::air;
            continue;
        } 
        mat1 = masses[m1].material;
        mat2 = masses[m2].material;
        // mat = materials::id_lookup(materials::get_composite_id(mat1.id, mat2.id));
        mat = materials::decode(mat1.encoding | mat2.encoding);

        Spring s = {m1, m2, dist, dist, mat};
        springs.push_back(s);
    }

    for(uint i = 0; i < triangulation.isBoundaryVertexFlags.size(); i++) {
        boundaryCount += triangulation.isBoundaryVertexFlags[i];
    }

    // end = std::chrono::high_resolution_clock::now();
    // execute_time = std::chrono::duration<float>(end - start).count();
    // printf("SPRINGS CREATED IN %f SECONDS\n", execute_time);

    ShiftX(*this);
    ShiftY(*this);

    updateBaseline();
}