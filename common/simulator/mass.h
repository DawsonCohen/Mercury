#ifndef __Mass_H__
#define __Mass_H__

#include <Eigen/Core>
#include "material.h"

struct Mass {
	uint	id;

	Eigen::Vector3f 	pos;
	Eigen::Vector3f 	protoPos;
	Material material = materials::bone;
	
	float mass = 1.0f;
	Eigen::Vector3f vel   = Eigen::Vector3f::Zero();
	Eigen::Vector3f acc   = Eigen::Vector3f::Zero();
	Eigen::Vector3f force = Eigen::Vector3f::Zero();

	bool	active = false;

	Mass() {}

	Mass(uint id, float x, float y, float z, float mass = 1.0f) :
	id(id), mass(mass)
	 {
		pos = Eigen::Vector3f(x, y, z);
		protoPos = pos;
	}

	Mass(uint id, Eigen::Vector3f pos, Material mat, float mass = 1.0f) :
	id(id), pos(pos), protoPos(pos), material(mat), mass(mass)
	{}
};

#endif