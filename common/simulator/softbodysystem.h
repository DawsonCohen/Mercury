#ifndef __SOFTBODYSYSTEM_H__
#define __SOFTBODYSYSTEM_H__

#include "material.h"

struct SimOptions {
	float dt;
	uint massesPerBlock;
	uint springsPerBlock;
	uint facesPerBlock;
	uint cellsPerBlock;
	uint boundaryMassesPerBlock;
	uint maxMasses;
	uint maxSprings;
	uint maxFaces;
	uint maxCells;
	uint compositeCount;
	short shiftskip;
	float drag;
	float damping;
	float relaxation;
	float s;
};

struct DevoOptions {
    uint maxReplacedSprings;
	uint maxSprings;
    uint springsPerElement;
    uint massesPerElement;
    uint replacedSpringsPerElement;
    uint compositeCount;
};

struct DeviceData {
	// MASS DATA
	float    *dPos, *dNewPos, *dVel;
	uint32_t *dMassMatEncodings;

	// SPRING DATA
	ushort	 *dPairs;
	uint32_t *dSpringMatEncodings;
	uint8_t  *dSpringMatIds;
	float	 *dLbars;
	uint     *dSpringIDs;
	float	 *dSpringStresses;
	
	// SPRING DEVO DATA
	ushort   *dRandomPairs;
	uint     *dSpringIDs_Sorted;
	float	 *dSpringStresses_Sorted;

	// FACE DATA
	ushort	 *dFaces;
	
	// CELL DATA
	ushort	 *dCells;
	float	 *dVbars;
	float	 *dMats;
	float	 *dCellStresses;
};
const uint  devoThreadsPerBlock = 256;

void setCompositeMats_id(float* compositeMats, uint count);

void setSimOpts(SimOptions opt);

void setCompositeMats_encoding(float* compositeMats, uint count);

void setDevoOpts(DevoOptions devoOpts);

void integrateBodies(DeviceData DeviceData, uint numElements, SimOptions opt, float time, uint step, bool integrateForce = false);

void getRandomInterPairs(int N, ushort* randomPairs, int min_value, int max_value, unsigned int seed);

void devoBodies(DeviceData deviceData, DevoOptions opt, float time);

#endif