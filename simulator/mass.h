#ifndef __Mass_H__
#define __Mass_H__

#include <stdint.h>
#include <iostream>
#include <Eigen/Core>

struct Color {
	float r;
	float g;
	float b;
	float a;

	constexpr Color(float r, float g, float b, float a) :
	r(r), g(g), b(b), a(a)
	{}

	constexpr Color operator*(float scalar) {
        return Color(r * scalar, g * scalar, b * scalar, a * scalar);
    }

	constexpr Color operator/(float scalar) {
		float rscalar = 1 / scalar;
        return Color(r * rscalar, g * rscalar, b * rscalar, a * rscalar);
    }
};

struct Mass {
	uint	id;

	Eigen::Vector3f 	pos;
	Eigen::Vector3f 	protoPos;
	Color color = Color(0.0f,0.0f,0.0f,1.0f);
	
	float mass = 1.0f;
	Eigen::Vector3f vel   = Eigen::Vector3f::Zero();
	Eigen::Vector3f acc   = Eigen::Vector3f::Zero();
	Eigen::Vector3f force = Eigen::Vector3f::Zero();

	bool	active = false;
};

#endif