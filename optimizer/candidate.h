#ifndef __CANDIDATE_H__
#define __CANDIDATE_H__

#include <algorithm>
#include <string>

#define MIN_FITNESS (float) 0

struct CandidatePair;

class Candidate {
friend class Evaluator;

protected:
    float   mFitness = 0;
    uint    mParetoLayer = 0;

public:
    int     mAge;

    Candidate() : mAge(0) {}

    Candidate(const Candidate& src): 
        mFitness(src.mFitness),
        mParetoLayer(src.mParetoLayer),
        mAge(src.mAge)
    {}


    friend void swap(Candidate& c1, Candidate& c2) {
        using std::swap;
        swap(c1.mFitness, c2.mFitness);
        swap(c1.mParetoLayer, c2.mParetoLayer);
        swap(c1.mAge,c2.mAge);
    }

    void IncrementAge() { mAge++; };

    float fitness() const { return mFitness; }
    uint paretoLayer() const { return mParetoLayer; }

    void setFitness(float fit) { mFitness = fit; }


    void Randomize();
    static void Random();
    void Mutate();
    static CandidatePair Crossover(const CandidatePair& parents);

    static void calcFitness(Candidate&);
    static float Distance(const CandidatePair& candidates);

    static std::vector<float> findDiversity(std::vector<Candidate> population);
    
    Candidate& operator=(Candidate src) {
        swap(*this, src);

        return *this;
    }

    bool operator < (const Candidate& R) const {
        return mFitness < R.mFitness;
    }

    bool operator > (const Candidate& R) const {
        return mFitness > R.mFitness;
    }

    bool operator <= (const Candidate& R) const {
        return !(*this > R);
    }

    bool operator >= (const Candidate& R) const {
        return !(*this < R);
    }

    std::string fitnessReadout() {
        return "fitness: " + std::to_string(mFitness) + "\tage: " + std::to_string(mAge);
    }
};

struct CandidatePair {
    Candidate first;
    Candidate second;
};

#endif