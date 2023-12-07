#ifndef __NN_ROBOT_H__
#define __NN_ROBOT_H__

// Evolvable Soft Body
#include <random>
#include <string>
#include <Eigen/Dense>
#include "SoftBody.h"
#include "EvoDevo/Util/util.h"

#define MIN_FITNESS (float) 0

namespace EvoDevo {

    class NNRobot : public SoftBody {
    private:
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

        void forward() {
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

            // tanh activation to position rows
            // x.topRows(output_size - MATERIAL_COUNT) = tanh(x.topRows(output_size - MATERIAL_COUNT)); 
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

                // printf("{%f,%f,%f}\n",masses[i].pos());

                Eigen::VectorXf mat_prob = material_probs.col(i);
                int maxIdx;
                mat_prob.maxCoeff(&maxIdx);
                masses[i].material = materials::matLookup(maxIdx);
            }
        }

    protected:
        std::vector<Eigen::MatrixXf> weights;
        static std::vector<unsigned int> hidden_sizes;
        static unsigned int num_layers;

        static float crossover_neuron_count;
        static int mutation_weight_count;
        static int springs_per_mass;
        static CrossoverDistribution crossover_distribution;
        static CrossoverType crossover_type;

        static bool randMassesFilled;
        static std::vector<Mass> randMasses;
        
        constexpr static unsigned int input_size = 3;
        constexpr static unsigned int output_size = 3 + MATERIAL_COUNT;

    public:
        static unsigned int maxMasses;
        static unsigned int maxSprings;

        static void fillRandMasses(unsigned int N) {
            if(!randMassesFilled) {
                int seed = 2383;
                std::default_random_engine gen = std::default_random_engine(seed);
                randMasses.clear();

                for(unsigned int i = 0; i < N; i++) {
                    float el = (uniform(gen) * M_PI) - M_PI/2;
                    float az = uniform(gen) * 2 * M_PI;
                    float r = uniform(gen);

                    float x = r * cos(el) * cos(az);
                    float z = r * cos(el) * sin(az);
                    float y = r * sin(el);

                    Mass m(i,x,y,z);
                    randMasses.push_back(m);
                }
            }
            randMassesFilled = true;
        }

        static void Configure(Config::NNRobot config) {
            NNRobot::crossover_neuron_count = config.crossover_neuron_count;
            NNRobot::mutation_weight_count = config.mutation_weight_count;
            NNRobot::springs_per_mass = config.springs_per_mass;
            
            NNRobot::num_layers = config.hidden_layer_sizes.size()+2;
            NNRobot::hidden_sizes = config.hidden_layer_sizes;

            NNRobot::crossover_distribution = config.crossover_distribution;
            NNRobot::crossover_type = config.crossover_type;

            NNRobot::maxMasses = config.massCount;
            NNRobot::maxSprings = config.massCount * config.springs_per_mass;

            fillRandMasses(config.massCount);
            randMassesFilled = true;
        }
        
        // NNRobot class configuration functions
        static void SetArchitecture(const std::vector<unsigned int>& hidden_layer_sizes = std::vector<unsigned int>{25,25}) {
            NNRobot::num_layers = hidden_sizes.size()+2;
            NNRobot::hidden_sizes = hidden_layer_sizes;
        }

        // Initializers
        void Build();
        static void BatchBuild(std::vector<NNRobot>& robots);

        NNRobot();

        NNRobot(std::vector<Eigen::MatrixXf> weights) :
            weights(weights)
        { Build(); }
        
        NNRobot(const NNRobot& src) : SoftBody(src),
            weights(src.weights)
        { }
        
        void Randomize() override;
        void Mutate() override;

        static CandidatePair<NNRobot> Crossover(const CandidatePair<NNRobot>& parents);

        static float Distance(const CandidatePair<NNRobot>& robots);

        friend void swap(NNRobot& r1, NNRobot& r2) {
            using std::swap;
            swap(r1.weights, r2.weights);
            swap(r1.num_layers, r2.num_layers);

            swap((SoftBody&) r1, (SoftBody&) r2);
        }

        NNRobot& operator=(NNRobot src) {
            swap(*this, src);

            return *this;
        }
        
        static std::vector<float> findDiversity(std::vector<NNRobot> pop);
    };

}

#endif