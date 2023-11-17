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
	void freeMemory();

public:
	Simulator() {};
	~Simulator();

	void Initialize(Config::Simulator = Config::Simulator());
	
	ElementTracker SetElement(const Element& element);
	std::vector<ElementTracker> SetElements(const std::vector<Element>& elements);
	ElementTracker AllocateElement(const Element& e);
	void Simulate(float sim_duration, bool trackStresses = false, bool trace = false, std::string tracefile = "trace.csv");
	void Devo();
	Element Collect(const ElementTracker& tracker);
	std::vector<Element> Collect(const std::vector<ElementTracker>& trackers);
	Element CollectElement(const ElementTracker& tracker);


	// void Simulate(std::vector<Mass>& masses, const std::vector<Spring>& springs);

	// Getter/setter time step in seconds
	float getDeltaT() const { return m_deltaT; }
	void setTimeStep(float dt) { m_deltaT = dt; }

	// Get simulated time elapsed in seconds
	float getTotalTime() const { return m_total_time; }

	void Reset() { m_total_time = 0.0f; }

protected:
	bool initialized = false;

	std::vector<Environment> mEnvironments;
    float m_total_time = 0;
    float m_deltaT = 0.001f;
	uint m_replacedSpringsPerElement = 32; // recommend multiple of 32 for warp
	// bool track_stresses = false;

	Mass*			massBuf;
	Spring*			springBuf;
	uint*			offsetBuf;
	Environment*	envBuf;

	// CPU data
	float    *m_hPos, *m_hVel;
	uint32_t *m_hMassMatEncodings;
	uint32_t *m_hSpringMatEncodings;
	uint8_t  *m_hSpringMatIds;
	float    *m_hCompositeMats_id;
	float    *m_hCompositeMats_encoding;
	ushort   *m_hPairs;
	float    *m_hLbars;
	ushort   *m_hMaxStressCount, *m_hMinStressCount;
	float    *m_hStresses;
	uint     *m_hSpringIDs;

	// GPU data
	float    *m_dPos, *m_dVel;
	uint32_t *m_dMassMatEncodings;
	uint32_t *m_dSpringMatEncodings;
	uint8_t  *m_dSpringMatIds;
	float    *m_dCompositeMats_encoding;
	float    *m_dCompositeMats_id;
	ushort	 *m_dPairs;
	ushort   *m_dRandomPairs;
	float	 *m_dLbars;
	ushort   *m_dMaxStressCount, *m_dMinStressCount;
	ushort   *m_dMinStressCount_Sorted;
	float	 *m_dStresses;
	uint     *m_dSpringIDs;
	uint     *m_dSpringIDs_Sorted;

	uint	 m_massesPerBlock = 0;
	uint	 m_springsPerBlock = 0;
	uint	 m_sharedMemSizeSim = 0;
	uint	 m_numBlocksSim = 0;

	uint  simThreadsPerBlock = 1024; // TODO: Remove
	const uint  devoThreadsPerBlock = 256;

	uint springsPerElement = 0;
	uint massesPerElement  = 0;
	uint maxElements       = 0;
	uint maxMasses 	       = 0;
	uint maxSprings        = 0;
	uint maxReplaced       = 0;
	uint maxEnvs           = 0;
	uint numElements       = 0;
	uint numMasses         = 0;
	uint numSprings        = 0;
	uint envCount          = 0;
	uint elementCount      = 0;

	Config::Simulator m_config;
};

#endif