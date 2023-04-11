#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include "spring.h"
#include "mass.h"

struct Environment
{
	float floor_stiffness = 1000000;
	Eigen::Vector3f g = glm::vec3(0.0f, 0.0f, 0.0f);
	float damping = 0.99f;
	//float friction = 0.5f;
	float drag = 1.204f;
};

#endif