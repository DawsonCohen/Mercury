#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "environment.h"
#include "element.h"
#include <memory>

struct ElementTracker {
	uint ID;
	Mass* mass_begin;
	Mass* mass_end;
	Mass* mass_chunk_end;
	Spring* spring_begin;
	Spring* spring_end;
	Spring* spring_chunk_end;
	uint* offset_begin;
	uint* offset_end;
	uint* offset_chunk_end;

	glm::vec3 mean_pos = glm::vec3(0.0f, 0.0f, 0.0f);
};

class Simulator {
	void _initialize();

public:
	Simulator(Element prototype, uint maxElements);
	// Simulator(float dt): step_period(dt) {}
	Simulator() {}
	~Simulator();

	void Initialize(Element prototype, uint maxElements);
	
	std::vector<ElementTracker> Allocate(const std::vector<Element>& element);
	ElementTracker AllocateElement(const Element& e);

	std::vector<Element> Collect(const std::vector<ElementTracker>& trackers);
	Element CollectElement(const ElementTracker& tracker);
	std::vector<ElementTracker> Simulate(std::vector<Element>& elements);

	// void Simulate(std::vector<Mass>& masses, const std::vector<Spring>& springs);

	// Getter/setter time step in seconds
	float getStepPeriod() const { return step_period; }
	void setTimeStep(float dt) { step_period = dt; }

	// Get simulated time elapsed in seconds
	float getTotalTime() const { return total_time; }

	// Getter/setter maximum simulation time in seconds
	float getMaxTime() const { return max_time; }
	void setMaxTime(float tmax) { max_time = tmax; }

	void Reset() { total_time = 0; }

	// std::tuple<std::vector<Mass>, std::vector<Spring>> element2simulatable(std::vector<Mass>, std::vector<Spring>);
	// void result2element(std::vector<Element>, std::vector<Mass>);


protected:
	bool initialized = false;

	std::vector<Environment> mEnvironments;
    float total_time = 0;
    float step_period = 0.0005f;
    float max_time = 10.0f;

	Mass*			massBuf;
	Spring*			springBuf;
	uint*			offsetBuf;
	Environment*	envBuf;

	// CPU data
	ushort *m_hPairs;
	float  *m_hMats;
	float  *m_hLbars;
	bool   *m_hActive;
	float  *m_hPos;
	float  *m_hVel;

	// GPU data
	float   *m_dPos[2], *m_dVel[2],
			*m_dMats;
	ushort	*m_dPairs;
	float	*m_dLbars;
	bool	*m_dActive;
	unsigned char m_currentRead,
			 	  m_currentWrite;

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