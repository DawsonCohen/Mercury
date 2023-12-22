#include "evpch.h"

#include <thread>
#include "NNRobot.h"
#include "triangulation.h"

#define min(a,b) a < b ? a : b

#define EPS 1e-1

/**
 * @file NNRobot.cpp
 * @brief Implementation file for the NNRobot class.
 * 
 * This file contains the implementation of the NNRobot class, which represents a neural network-based robot.
 * It includes functions for forward propagation, randomization, mutation, crossover, distance calculation,
 * diversity calculation, and building the robot's structure.
 */
namespace EvoDevo {

    std::vector<unsigned int> NNRobot::hidden_sizes = std::vector<unsigned int>{25,25};

    std::vector<Mass> NNRobot::randMasses;
    bool NNRobot::randMassesFilled = false;

    unsigned int NNRobot::num_layers = 4;
    float NNRobot::crossover_neuron_count = .2;
    int NNRobot::mutation_weight_count = 10;
    int NNRobot::springs_per_mass = 25;
    unsigned int NNRobot::maxMasses = 1365;
    unsigned int NNRobot::maxSprings = NNRobot::maxMasses * NNRobot::springs_per_mass;
    CrossoverDistribution NNRobot::crossover_distribution = CROSS_DIST_BINOMIAL;
    CrossoverType NNRobot::crossover_type = CROSS_CONTIGUOUS;

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

    Eigen::MatrixXf relu(const Eigen::MatrixXf& x) {
            return x.array().max(0);
        }

    Eigen::MatrixXf tanh(const Eigen::MatrixXf& x) {
        return x.array().tanh();
    }

    Eigen::MatrixXf softmax(const Eigen::MatrixXf& input) {
        Eigen::MatrixXf output(input.rows(), input.cols());
        for (int j = 0; j < input.cols(); j++) {
            Eigen::VectorXf col = input.col(j);
            col.array() -= col.maxCoeff(); // subtract max for numerical stability
            col = col.array().exp();
            output.col(j) = col / col.sum();
        }
        return output;
    }

    Eigen::MatrixXf addBias(const Eigen::MatrixXf& A) {
        Eigen::MatrixXf B(A.rows()+1, A.cols());
        B.topRows(A.rows()) = A;
        B.row(A.rows()).setOnes();
        return B;
    }

    void NNRobot::forward() {            
        if(!randMassesFilled) {
            fillRandMasses(maxMasses);
        }
        masses = randMasses;
        
        Eigen::MatrixXf input(input_size, masses.size());

        for(size_t i = 0; i < masses.size(); i++) {
            Mass m = masses[i];
            input.col(i) << m.protoPos.x(), m.protoPos.y(), m.protoPos.z();
        }

        Eigen::MatrixXf x = input;
        for (unsigned int i = 0; i < num_layers-2; i++) {
            // x = addBias(x);
            x = weights[i] * x;
            x = relu(x);
        }

        // x = addBias(x);
        x = weights[num_layers-2] * x;

        float maxNorm = 0.0f;
        for (int i = 0; i < x.cols(); ++i) {
            float norm = x.col(i).head(output_size - MATERIAL_COUNT).norm();
            if(maxNorm < norm) maxNorm = norm;
        }
        x.topRows(output_size - MATERIAL_COUNT) = 10 * x.topRows(output_size - MATERIAL_COUNT) / maxNorm;

        
        // softmax activation to material rows
        x.bottomRows(MATERIAL_COUNT) = softmax(x.bottomRows(MATERIAL_COUNT)); 

        Eigen::MatrixXf output = x;
        Eigen::MatrixXf positions = output.topRows(3);
        Eigen::MatrixXf material_probs = output.bottomRows(MATERIAL_COUNT);
        
        for(uint i = 0; i < masses.size(); i++) {
            // Eigen::Vector3f = psoitions.col(i);
            masses[i].pos = masses[i].protoPos = positions.col(i);

            Eigen::VectorXf mat_prob = material_probs.col(i);
            int maxIdx;
            mat_prob.maxCoeff(&maxIdx);
            masses[i].material = materials::matLookup(maxIdx);
        }
    }

    /**
     * Produces random weight matrices to prdocue a random NNRobot
     */
    void NNRobot::Randomize() {
        for(unsigned int i = 0; i < weights.size(); i++) {
            weights[i] = Eigen::MatrixXf::Random(weights[i].rows(), weights[i].cols());
        }

        mAge = 0;
    }

    /**
     * @brief Mutates the weights of the neural network robot.
     * 
     * This function randomly assigns a new random value to randomly selected weights
     * The new value is generated from a uniform distribution between -1 and 1.
     */
    void NNRobot::Mutate() {
        for(int i = 0; i < mutation_weight_count; i++) {
            int layer = rand() % weights.size();
            int row = rand() % weights[layer].rows();
            int col = rand() % weights[layer].cols();

            weights[layer](row,col) = uniform(gen)*2-1;
        }
    }

    
    /**
     * Performs crossover operation on two NNRobot parents to produce children.
     * Swaps the weights of a randomly selected node (i.e. row of random weight matrix)
     * @param parents The pair of NNRobot parents for crossover.
     * @return The pair of NNRobot children generated from crossover.
     */
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
        
        // // TODO: this is a significant slowdown
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
        unsigned int robots_per_thread = (robots.size() + active_threads - 1) / active_threads;
        
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

    /**
     * @brief Builds the robot by performing alpha shape triangulation on the masses.
     *
     * This function clears the existing robot, performs alpha shape triangulation on the masses,
     * and generates the necessary springs, faces, and cells for the robot. The resulting robot
     * is then shifted in the X and Y directions, and the baseline is updated
     * 
     * Note: this must be called before simulation
     */
    void NNRobot::Build() {
        Clear();

        forward();

        /**
         * @brief Perform alpha shape triangulation on the masses.
         *
         * This function uses the Triangulation::AlphaShape method to perform alpha shape triangulation
         * on the given masses.
         *
         * @param masses The masses to perform triangulation on.
         * @return The triangulated shape.
         */
        auto triangulation = Triangulation::AlphaShape(this->masses);
        // auto triangulation = Triangulation::KNN(this->masses,springs_per_mass);

        for (auto edge : triangulation.edges) {
            uint16_t m1 = edge.v1,
                    m2 = edge.v2;
            float dist = edge.dist;
            Material mat1, mat2, mat;

            if(dist < EPS) {
                mat = materials::air;
                continue;
            } 
            mat1 = masses[m1].material;
            mat2 = masses[m2].material;
            mat = materials::decode(mat1.encoding | mat2.encoding);

            Spring s = {m1, m2, dist, dist, mat};
            springs.push_back(s);
        }

        for (auto facet : triangulation.facets) {
            uint16_t m1 = facet.v1,
                    m2 = facet.v2,
                    m3 = facet.v3;
            Face f = {m1, m2, m3};
            faces.push_back(f);
        }

        for (auto cell : triangulation.cells) {
            uint16_t v1 = cell.v1,
                    v2 = cell.v2,
                    v3 = cell.v3,
                    v4 = cell.v4;
            Mass m0 = masses[v1],
                m1 = masses[v2],
                m2 = masses[v3],
                m3 = masses[v4];
            Material mat1, mat2, mat3, mat4, mat;

            mat1 = masses[v1].material;
            mat2 = masses[v2].material;
            mat3 = masses[v3].material;
            mat4 = masses[v4].material;
            mat = materials::avg({mat1, mat2, mat3, mat4});

            float volume = ((m1.pos-m0.pos).cross(m2.pos-m0.pos)).dot(m3.pos-m0.pos) / 6.0f;

            Cell c = {v1, v2, v3, v4, volume, mat};
            cells.push_back(c);
        }

        ShiftX();
        ShiftY();

        updateBaseline();
    }
}