#ifndef __Mass_H__
#define __Mass_H__

#include <Eigen/Core>
#include <iomanip>
#include <iostream>
#include <limits>
#include "material.h"

struct Mass {
	uint	id;

	Eigen::Vector3f 	pos;
	Eigen::Vector3f 	protoPos;
	Eigen::Vector3f		vel = Eigen::Vector3f::Zero();
	Material material = materials::bone;
	
	float mass = 1.0f;

	bool	active = false;

	Mass() {}

	Mass(uint id, float x, float y, float z, float mass = 1.0f, Material mat = materials::bone) :
	id(id), material(mat), mass(mass)
	 {
		pos = Eigen::Vector3f(x, y, z);
		protoPos = pos;
	}

	Mass(uint id, Eigen::Vector3f pos, Material mat, float mass = 1.0f) :
	id(id), pos(pos), protoPos(pos), material(mat), mass(mass)
	{}

	friend std::ostream& operator<<(std::ostream& out, const Mass& m) {
		out << std::setprecision(9);
		return out << m.id << "," << m.protoPos.x() << "," << m.protoPos.y() << "," << m.protoPos.z() << "," << m.mass << "," << (unsigned int) m.material.encoding;
	}
};

#endif