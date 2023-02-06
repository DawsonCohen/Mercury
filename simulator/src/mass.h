#ifndef __Mass_H__
#define __Mass_H__

#include <stdint.h>
#include <iostream>
#include "vector_types.h"
#include <glm/glm.hpp>

struct Mass {
	uint	id;

	glm::vec3 	pos;
	glm::vec3 	protoPos;
	
	float 	mass = 1.0f;
	float3 	vel = {0.0f, 0.0f, 0.0f};
	float3 	acc = {0.0f, 0.0f, 0.0f};
	float3 	force = {0.0f, 0.0f, 0.0f};
	glm::vec4 color = glm::vec4(0.0f,0.0f,0.0f,1.0f);

	bool	active = false;
};

#endif