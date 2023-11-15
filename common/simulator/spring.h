#ifndef __SPRING_H__
#define __SPRING_H__
#include <vector>
#include "mass.h"
#include "stdio.h"
#include "math.h"

struct Spring {
	uint16_t m0;
	uint16_t m1;
	float rest_length;
	float mean_length;
	Material material;

	friend std::ostream& operator<<(std::ostream& out, const Spring& s) {
		out << std::setprecision(9);
		return out << s.m0 << "," << s.m1 << "," << (double) s.rest_length << "," << (double) s.mean_length << "," << (unsigned int) s.material.id;
	}
};

#endif