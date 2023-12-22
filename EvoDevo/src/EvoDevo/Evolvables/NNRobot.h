#ifndef __NN_ROBOT_H__
#define __NN_ROBOT_H__

// Evolvable Soft Body
#include "EvoDevo/Core/Base.h"
#include <random>
#include <string>
#include <Eigen/Dense>
#include "SoftBody.h"
#include "EvoDevo/Util/util.h"

#define MIN_FITNESS (float) 0

namespace EvoDevo {

    class NNRobot : public SoftBody {
        void forward();
        
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

                    Mass m (i,{x,y,z});
                    randMasses.push_back(m);
                }
            }
            randMassesFilled = true;
        }
    public:
        static unsigned int maxMasses;
        static unsigned int maxSprings;


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