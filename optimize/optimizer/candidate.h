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
    uint    mAge = 0;
    bool    mParentFlag = 0;
    bool    mValid = true;

public:

    Candidate() : mAge(0) {}
    virtual ~Candidate() {}

    Candidate(const Candidate& src): 
        mFitness(src.mFitness),
        mParetoLayer(src.mParetoLayer),
        mAge(src.mAge),
        mParentFlag(src.mParentFlag),
        mValid(src.mValid)
    {}


    friend void swap(Candidate& c1, Candidate& c2) {
        using std::swap;
        swap(c1.mFitness, c2.mFitness);
        swap(c1.mParetoLayer, c2.mParetoLayer);
        swap(c1.mAge,c2.mAge);
        swap(c1.mValid,c2.mValid);
        swap(c1.mParentFlag,c2.mParentFlag);
    }

    void IncrementAge() { mAge++; };

    float fitness() const { return mFitness; }
    bool isValid() const { return mValid; }
    bool isParent() const { return mParentFlag; }
    uint age() const { return mAge; }
    uint paretoLayer() const { return mParetoLayer; }

    void setFitness(float fit) { mFitness = fit; }
    void setIsParent(bool on) { mParentFlag = on; }


    // void Randomize();
    virtual void updateFitness() = 0;
    virtual void Randomize() = 0;
    virtual void Mutate() = 0;
    virtual void Reset() = 0;
    virtual void Clear() = 0;

    bool operator < (const Candidate& R) const {
        if(mParetoLayer > R.mParetoLayer)
            return true;
        else if(mParetoLayer == R.mParetoLayer)
            return mFitness < R.mFitness;

        return false;
    }

    bool operator > (const Candidate& R) const {
        if(mParetoLayer < R.mParetoLayer)
            return true;
        else if(mParetoLayer == R.mParetoLayer)
            return mFitness > R.mFitness;
        
        return false;
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