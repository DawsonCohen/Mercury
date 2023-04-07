#ifndef __SOFT_BODY_H__
#define __SOFT_BODY_H__

#include "element.h"
#include "candidate.h"
#include <Eigen/Geometry>
#include <memory>

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
	float	mVolume = 0;

protected:
	std::vector<Material> mDirectEncoding;
	std::vector<MaterialRadii> mRadiiEncoding;

public:
	SoftBody() { }

	SoftBody(const SoftBody&);

	friend void swap(SoftBody& s1, SoftBody& s2) {
		using std::swap;
		swap((Candidate&) s1, (Candidate&) s2);
		swap(s1.masses, s2.masses);
		swap(s1.springs, s2.springs);
		swap(s1.mDirectEncoding, s2.mDirectEncoding);
		swap(s1.mRadiiEncoding, s2.mRadiiEncoding);
	}

	void rotate(float deg, Eigen::Vector3f& axis);
	void translate(Eigen::Vector3f& translation);

	const std::vector<Mass>& getMasses() const { return masses; };
	const std::vector<Spring>& getSprings() const { return springs; }

	float getSimTime() const { return sim_time; }
	float getTotalSimTime() const { return total_sim_time; }
    void incrementSimTime(float dt) { sim_time += dt; total_sim_time += dt; }
    void resetSimTime() { sim_time = 0; }

	void Update(Element e) { 
		masses = e.masses; 
		springs = e.springs; 
	}
    void SimReset();
	void Reset();
	void Clear();
	void updateMesh();
	void updateVolume(float v) { mVolume = v; }

	void append(SoftBody src);

	void addMass(Mass mass) { 
		masses.push_back(mass);
	}

	void addSpring(Spring& s) {
		if(s.material != materials::air) {
			masses[s.m0].active = true;
			masses[s.m1].active = true;
			masses[s.m0].color = s.material.color;
			masses[s.m1].color = s.material.color;
		}
		springs.push_back(s);
	}

	void setMasses(std::vector<Mass> _masses) {
		masses.resize(_masses.size());
		for(uint i = 0; i < masses.size(); i++) {
			masses[i] = _masses[i];
		}
	}

	void setSprings(std::vector<Spring> _springs) {
		springs.clear();
		for(Spring& s : _springs) {
			if(s.material != materials::air) {
				masses[s.m0].active = true;
				masses[s.m1].active = true;
				masses[s.m0].color = s.material.color;
				masses[s.m1].color = s.material.color;
			}
			springs.push_back(s);
		}
	}

	void printObjectPositions();
	
	SoftBody& operator=(SoftBody src)
	{
		swap(*this, src);
		
		return *this;
	}
};

#endif