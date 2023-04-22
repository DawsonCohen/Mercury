#ifndef __NN_ROBOT_H__
#define __NN_ROBOT_H__

// Evolvable Soft Body
#include <random>
#include <string>
#include <Eigen/Dense>
#include "SoftBody.h"
#include "config.h"

#define MIN_FITNESS (float) 0

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
        x.topRows(output_size - MATERIAL_COUNT) = tanh(x.topRows(output_size - MATERIAL_COUNT)); 
        
        // softmax activation to material rows
        x.bottomRows(MATERIAL_COUNT) = softmax(x.bottomRows(MATERIAL_COUNT)); 

        Eigen::MatrixXf output = x;
        Eigen::MatrixXf positions = output.topRows(3);
        Eigen::MatrixXf material_probs = output.bottomRows(MATERIAL_COUNT);
        
        for(uint i = 0; i < masses.size(); i++) {
            masses[i].pos = masses[i].protoPos = positions.col(i);

            Eigen::VectorXf mat_prob = material_probs.col(i);
            int maxIdx;
            mat_prob.maxCoeff(&maxIdx);
            masses[i].material = materials::matLookup(maxIdx);
        }
    }

protected:
    std::vector<Eigen::MatrixXf> weights;
    static std::vector<int> hidden_sizes;
    static unsigned int num_layers;

    static int crossover_neuron_count;
    static int mutation_weight_count;
    static int springs_per_mass;
    
    constexpr static unsigned int input_size = 3;
    constexpr static unsigned int output_size = 3 + MATERIAL_COUNT;
    
    
public:
    // TODO: more elegant solution for simulator initialization
    unsigned int maxMasses;
    unsigned int maxSprings;

    static void Configure(Config::NNRobot config) {
        NNRobot::crossover_neuron_count = config.crossover_neuron_count;
        NNRobot::mutation_weight_count = config.mutation_weight_count;
        NNRobot::springs_per_mass = config.springs_per_mass;
        
        NNRobot::num_layers = config.hidden_layer_sizes.size()+2;
        NNRobot::hidden_sizes = config.hidden_layer_sizes;
    }
    
    // NNRobot class configuration functions
    static void SetArchitecture(const std::vector<int>& hidden_layer_sizes = std::vector<int>{25,25}) {
        NNRobot::num_layers = hidden_sizes.size()+2;
        NNRobot::hidden_sizes = hidden_layer_sizes;
    }

    // Initializers
    void Build();
	static void BatchBuild(std::vector<NNRobot>& robots); // TODO

    NNRobot(const unsigned int num_masses = 1728);

    NNRobot(std::vector<Eigen::MatrixXf> weights, const unsigned int num_masses=1728) :
        weights(weights), maxMasses(num_masses), maxSprings(num_masses*25)
    { Build(); }
    
    NNRobot(const NNRobot& src) : SoftBody(src),
        weights(src.weights), maxMasses(src.maxMasses), maxSprings(src.maxSprings)
    { }
    

    // Getters
    unsigned int volume() const { return mVolume; }
    Eigen::Vector3f COM() const { return mBaseCOM; }

    void Randomize();
    void Mutate();

    static CandidatePair<NNRobot> Crossover(const CandidatePair<NNRobot>& parents);

    static float Distance(const CandidatePair<NNRobot>& robots);

    std::string Encode() const;
	void Decode(const std::string& filename);

    friend void swap(NNRobot& r1, NNRobot& r2) {
        using std::swap;
        swap(r1.weights, r2.weights);
        swap(r1.num_layers, r2.num_layers);
        swap(r1.mVolume, r2.mVolume);
        swap(r1.mBaseCOM, r2.mBaseCOM);
        swap(r1.mLength, r2.mLength);

        swap((SoftBody&) r1, (SoftBody&) r2);
    }

    NNRobot& operator=(NNRobot src) {
        swap(*this, src);

        return *this;
    }

    bool operator < (const NNRobot& R) const {
        if(mParetoLayer > R.mParetoLayer)
            return true;
        else if(mParetoLayer == R.mParetoLayer)
            return mFitness < R.mFitness;

        return false;
    }

    bool operator > (const NNRobot& R) const {
        if(mParetoLayer < R.mParetoLayer)
            return true;
        else if(mParetoLayer == R.mParetoLayer)
            return mFitness > R.mFitness;
        
        return false;
    }

    bool operator <= (const NNRobot& R) const {
        if(mParetoLayer < R.mParetoLayer)
            return false;
        else if(mParetoLayer == R.mParetoLayer)
            return !(mFitness > R.mFitness);

        return true;
    }

    bool operator >= (const NNRobot& R) const {
        if(mParetoLayer > R.mParetoLayer)
            return false;
        else if(mParetoLayer == R.mParetoLayer)
            return !(mFitness < R.mFitness);

        return true;
    }

    /*bool operator < (const NNRobot& R) const {
        return mParetoLayer > R.mParetoLayer;
    }

    bool operator > (const NNRobot& R) const {
        if(mParetoLayer < R.mParetoLayer)
            return true;
        else if(mParetoLayer == R.mParetoLayer)
            return mFitness > R.mFitness;
        
        return false;
    }

    bool operator <= (const NNRobot& R) const {
        return !(mFitness > R.mFitness);
    }

    bool operator >= (const NNRobot& R) const {
        return !(mFitness < R.mFitness);
    }*/
    
    static std::vector<float> findDiversity(std::vector<NNRobot> pop);
};

#endif
