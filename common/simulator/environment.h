#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "spring.h"
#include "mass.h"

enum EnvironmentType {
    ENVIRONMENT_LAND,
    ENVIRONMENT_WATER
};

struct Environment
{
	EnvironmentType type;
	float floor_stiffness;
	float g;
	float friction;
	float drag;
	float damping = 0.99f;
};

const Environment EnvironmentLand {
	ENVIRONMENT_LAND,
	1000000.0f,
	9.81f,
	0.8f,
	0.0f,
	0.99f
};

const Environment EnvironmentWater {
	ENVIRONMENT_WATER,
	0.0,
	0.0f,
	0.0f,
	1.204f,
	0.99f
};

#endif