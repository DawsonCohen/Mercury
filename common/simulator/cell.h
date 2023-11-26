#ifndef __CELL_H__
#define __CELL_H__
#include "mass.h"
#include "stdio.h"

struct Cell {
	uint16_t m0;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
	float mean_volume;
	Material material;

	friend std::ostream& operator<<(std::ostream& out, const Cell& c) {
		out << std::setprecision(9);
		return out << c.m0 << "," << c.m1 << "," << "," << c.m2 << "," << c.m3 << "," <<
				(double) c.mean_volume << "," << (unsigned int) c.material.id;
	}
};

#endif