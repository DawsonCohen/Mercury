#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "spring.h"
#include "mass.h"
#include "vector_types.h"

struct Environment
{
	float floor_stiffness = 1000000;
	float3 g = {0, -9.81, 0};
	float damping = .99;
	float friction = .4;
	float zeta = .8;
};

#endif