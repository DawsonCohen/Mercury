#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "spring.h"
#include "mass.h"
#include "vector_types.h"

struct Environment
{
	float floor_stiffness = 1000000;
	float3 g = {0.0f, -9.81f, 0.0f};
	float damping = 0.99f;
	float friction = 1.0f;
};

#endif