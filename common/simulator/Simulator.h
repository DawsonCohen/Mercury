#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "environment.h"
#include "element.h"
#include "config.h"
#include <memory>

// TODO: Face statistics if necessary??
struct ElementTracker {
	uint ID;
	Mass* mass_begin;
	Mass* mass_end;
	Spring* spring_begin;
	Spring* spring_end;
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
    float m_deltaT = 0.0001f;
	uint m_replacedSpringsPerElement = 32; // recommend multiple of 32 for warp
	// bool track_stresses = false;

	Mass*           massBuf;
	Spring*         springBuf;
	Face*           faceBuf;
	Cell*           cellBuf;
	Environment*	envBuf;

	// ----------- CPU data --------------
	float    *m_hCompositeMats_id;
	float    *m_hCompositeMats_encoding;

	// MASS DATA
	float    *m_hPos, *m_hVel;
	uint32_t *m_hMassMatEncodings;
	
	// SPRING DATA
	ushort   *m_hPairs;
	uint32_t *m_hSpringMatEncodings;
	uint8_t  *m_hSpringMatIds;
	float    *m_hLbars;
	uint     *m_hSpringIDs;
	float    *m_hSpringStresses;

	// FACE DATA
	ushort   *m_hFaces;

	// CELL DATA
	ushort   *m_hCells;
	float    *m_hVbars;
	float    *m_hMats;
	float    *m_hCellStresses;

	// ----------- GPU data --------------
	// MASS DATA
	float    *m_dPos, *m_dNewPos, *m_dVel;
	uint32_t *m_dMassMatEncodings;

	// SPRING DATA
	ushort	 *m_dPairs;
	uint32_t *m_dSpringMatEncodings;
	uint8_t  *m_dSpringMatIds;
	float	 *m_dLbars;
	uint     *m_dSpringIDs;
	float	 *m_dSpringStresses;
	
	// SPRING DEVO DATA
	ushort   *m_dRandomPairs;
	uint     *m_dSpringIDs_Sorted;
	float	 *m_dSpringStresses_Sorted;

	// FACE DATA
	ushort	 *m_dFaces;

	// CELL DATA
	ushort	 *m_dCells;
	float	 *m_dVbars;
	float	 *m_dMats;
	float	 *m_dCellStresses;
	// -------------------------

	uint	 m_massesPerBlock = 0;
	uint	 m_springsPerBlock = 0;
	uint	 m_facesPerBlock = 0;
	uint	 m_cellsPerBlock = 0;
	uint	 m_sharedMemSizeSim = 0;
	uint	 m_numBlocksSim = 0;

	uint  simThreadsPerBlock = 1024;
	const uint  devoThreadsPerBlock = 256;

	uint massesPerElement  = 0;
	uint boundaryMassesPerElement  = 0;
	uint springsPerElement = 0;
	uint facesPerElement   = 0;
	uint cellsPerElement   = 0;

	uint maxElements       = 0;
	uint maxMasses 	       = 0;
	uint maxSprings        = 0;
	uint maxFaces          = 0;
	uint maxCells          = 0;
	uint maxReplaced       = 0;
	uint maxEnvs           = 0;
	uint numElements       = 0;
	uint numMasses         = 0;

	uint numSprings        = 0;
	uint numFaces          = 0;
	uint numCells          = 0;
	uint envCount          = 0;
	uint elementCount      = 0;

	Config::Simulator m_config;
};

#endif