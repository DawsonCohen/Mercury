#ifndef __CANDIDATE_H__
#define __CANDIDATE_H__

#include <algorithm>
#include <string>

#define MIN_FITNESS (float) 0

template<typename T>
struct CandidatePair {
    T first;
    T second;
};

class Candidate {
template<typename U>
friend class Evaluator;

protected:
    float   mFitness = 0;
    uint    mParetoLayer = 0;

public:
    uint     mAge;
    bool    parentFlag = 0;
    bool    valid = true;

    Candidate() : mAge(0) {}
    virtual ~Candidate() {}

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
    uint age() const { return mAge; }
    uint paretoLayer() const { return mParetoLayer; }

    void setFitness(float fit) { mFitness = fit; }


    // void Randomize();
    virtual void updateFitness() = 0;
    virtual void Randomize() = 0;
    virtual void Mutate() = 0;
    virtual void Reset() = 0;
    virtual void Clear() = 0;

    bool operator < (const Candidate& R) const {
        return mParetoLayer > R.mParetoLayer;
    }

    bool operator > (const Candidate& R) const {
        return mParetoLayer < R.mParetoLayer;
    }

    bool operator <= (const Candidate& R) const {
        return !(*this > R);
    }

    bool operator >= (const Candidate& R) const {
        return !(*this < R);
    }

    virtual bool dominates(const Candidate&) const = 0;

    std::string fitnessReadout() {
        return "fitness: " + std::to_string(mFitness) + "\tage: " + std::to_string(mAge);
    }
};

#endif