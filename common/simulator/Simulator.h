#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "environment.h"
#include "element.h"
#include "config.h"
#include <memory>

struct ElementTracker {
	uint ID;
	Mass* mass_begin;
	Mass* mass_end;
	Spring* spring_begin;
	Spring* spring_end;
	uint* offset_begin;
	uint* offset_end;
};

class Simulator {
	void _initialize();

public:
	Simulator(Element prototype, uint maxElements);
	Simulator() {}
	~Simulator();

	void Initialize(Element prototype, uint maxElements, Config::Simulator config = Config::Simulator());
	void Initialize(uint massesPerElement, uint springsPerElement, uint maxElements, Config::Simulator);
	
	std::vector<ElementTracker> Allocate(const std::vector<Element>& element);
	ElementTracker AllocateElement(const Element& e);

	std::vector<Element> Collect(const std::vector<ElementTracker>& trackers);
	Element CollectElement(const ElementTracker& tracker);
	std::vector<ElementTracker> Simulate(std::vector<Element>& elements);

	// void Simulate(std::vector<Mass>& masses, const std::vector<Spring>& springs);

	// Getter/setter time step in seconds
	float getDeltaT() const { return deltaT; }
	void setTimeStep(float dt) { deltaT = dt; }

	// Get simulated time elapsed in seconds
	float getTotalTime() const { return total_time; }

	// Getter/setter maximum simulation time in seconds
	float getMaxTime() const { return max_time; }
	void setMaxTime(float tmax) { max_time = tmax; }
	void setDevoTime(float tmax) { devo_time  = tmax; }
	void setDevoCycles(uint cycles) { max_devo_cycles = cycles; }

	void Reset() { total_time = 0.0f; }

protected:
	bool initialized = false;

	std::vector<Environment> mEnvironments;
    float total_time = 0;
    float deltaT = 0.001f;
    float max_time = 10.0f;
    float devo_time = 1.0f;
    uint max_devo_cycles = 100;
	bool track_stresses = false;
	bool devo = false;

	Mass*			massBuf;
	Spring*			springBuf;
	uint*			offsetBuf;
	Environment*	envBuf;

	// CPU data
	ushort   *m_hPairs;
	float  *m_hMats;
	float  *m_hLbars;
	float  *m_hPos;
	float  *m_hVel;
	ushort *m_hMaxStressCount, *m_hMinStressCount;
	float  *m_hStresses;
	uint   *m_hSpringIDs;

	// GPU data
	float   *m_dPos[2], *m_dVel[2],
			*m_dMats;
	ushort	*m_dPairs;
	float	*m_dLbars;
	ushort  *m_dMaxStressCount, *m_dMinStressCount;
	float	*m_dStresses;
	uint    *m_dSpringIDs;

	unsigned char m_currentRead,
			 	  m_currentWrite;

	const uint  threadsPerBlock = 1024;

	uint springsPerElement = 0;
	uint massesPerElement  = 0;
	uint maxElements       = 0;
	uint maxMasses 	       = 0;
	uint maxSprings        = 0;
	uint maxEnvs           = 0;
	uint numElements       = 0;
	uint numMasses         = 0;
	uint numSprings        = 0;
	uint envCount          = 0;
	uint elementCount      = 0;
};

#endif