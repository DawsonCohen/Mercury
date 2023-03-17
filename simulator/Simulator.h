#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "environment.h"
#include "element.h"
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

struct massToSpringMapInfo {
	ushort massCount;
	ushort firstSpring;
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

protected:
	bool initialized = false;

	std::vector<Environment> mEnvironments;
    float total_time = 0;
    float step_period = 0.0005f;
    float max_time = 10.0f;

	Mass			*massBuf;
	Spring			*springBuf,
					*lSpringBuf,
					*rSpringBuf;
	uint			*offsetBuf;
	massToSpringMapInfo	*lmassMapBuf,
						*rmassMapBuf;
	Environment		*envBuf;
	
	// CPU data
	ushort	*m_hPairs,
			*m_hlPairs,
			*m_hrPairs;
		
	float	*m_hMats,
			*m_hlMats,
			*m_hrMats;
	float	*m_hLbars,
			*m_hlLbars,
			*m_hrLbars;
	float	*m_hPos;
	float	*m_hVel;
	ushort	*m_hMaxStressCount, *m_hMinStressCount;
	float	*m_hStresses;
	uint	*m_hSpringIDs;
	ushort	*m_hlCounts, *m_hrCounts;

	// GPU data
	float   *m_dPos[2], *m_dVel[2],
			*m_dMats,
			*m_dlMats,
			*m_drMats;
	ushort	*m_dPairs,
			*m_dlPairs,
			*m_drPairs;
	float	*m_dLbars,
			*m_dlLbars,
			*m_drLbars;
	ushort  *m_dMaxStressCount, *m_dMinStressCount;
	float	*m_dStresses;
	uint    *m_dSpringIDs;
	ushort	*m_dlCounts, *m_drCounts;

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