#ifndef __SOFT_BODY_H__
#define __SOFT_BODY_H__

#include "EvoDevo/Simulator/element.h"
#include "EvoDevo/Optimizer/candidate.h"
#include <Eigen/Geometry>
#include <random>
#include <memory>

namespace EvoDevo {

struct MaterialRadii {
	Material M;
	float radius;
};

struct DirectSoftBodyEncoding {
	std::vector<Material> mEncoding;
};

struct RadiiSoftBodyEncoding {
	std::vector<MaterialRadii> mEncoding;
};

class SoftBody : public Element, public Candidate {
protected:
	static unsigned seed;
    static std::default_random_engine gen;
    static std::uniform_real_distribution<> uniform;

	float   mVolume = 0.0f;
    float   mLength = 1.0f;
	Eigen::Vector3f mCOM;
    Eigen::Vector3f mBaseCOM;
	Eigen::Vector3f mClosestPos;

public:
	SoftBody() { }
	~SoftBody() override { }

	SoftBody(const Element& src) : Element(src) {
		Update(src);
		updateBaseline();
	}
	SoftBody(const SoftBody& src) :
	Element(src), Candidate(src),
	mVolume(src.mVolume), mLength(src.mLength), mBaseCOM(src.mBaseCOM)
	{}

	static void BatchBuild(std::vector<SoftBody>);

    std::string Encode() const;
	void Decode(const std::string& filename);

	friend void swap(SoftBody& s1, SoftBody& s2) {
		using std::swap;
		swap((Candidate&) s1, (Candidate&) s2);
		swap((Element&) s1, (Element&) s2);
		swap(s1.mVolume, s2.mVolume);
        swap(s1.mBaseCOM, s2.mBaseCOM);
        swap(s1.mLength, s2.mLength);
	}

	void rotate(float deg, Eigen::Vector3f& axis);
	void translate(Eigen::Vector3f& translation);

	Eigen::Vector3f getCOM() const { return mCOM; }
	Eigen::Vector3f getBaseCOM() const { return mBaseCOM; }
	Eigen::Vector3f getClosestPos() const { return mClosestPos; }
    void incrementSimTime(float dt) { sim_time += dt; total_sim_time += dt; }
    void resetSimTime() { sim_time = 0; }

	void updateBaseline() {
		updateLength();
		updateCOM();
		mBaseCOM = mCOM;
	}

	void Reset() override;
	void Clear() override;

	void Randomize() override {}
	void Mutate() override {}

	void updateVolume(float v) { mVolume = v; }

	virtual bool dominates(const Candidate& C) const override {
		return 	(mFitness >= C.fitness() &&
				 mAge <= C.age()) &&
				(mFitness > C.fitness() ||
				 mAge < C.age());
	}

	void addMass(Mass mass) { 
		masses.push_back(mass);
	}

	void addSpring(Spring& s) {
		springs.push_back(s);
	}

	void setMasses(std::vector<Mass> _masses) {
		masses.clear();
		for(size_t i = 0; i < _masses.size(); i++) {
			masses.push_back(_masses[i]);
		}
	}

	void setSprings(std::vector<Spring> _springs) {
		springs.clear();
		for(Spring& s : _springs) {
			springs.push_back(s);
		}
	}

	void updateCOM();
    void updateClosestPos();
    void updateLength();
	
    void updateFitness() override;
	
	void ShiftX(); void ShiftY();

	void printObjectPositions();
	
	SoftBody& operator=(SoftBody src)
	{
		swap(*this, src);
		
		return *this;
	}
};

}

#endif