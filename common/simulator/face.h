#ifndef __FACE_H__
#define __FACE_H__
#include "mass.h"
#include "stdio.h"

struct Face {
	uint16_t m0;
    uint16_t m1;
    uint16_t m2;
	bool active = true;

	friend std::ostream& operator<<(std::ostream& out, const Face& f) {
		out << std::setprecision(9);
		return out << f.m0 << "," << f.m1 << "," << f.m2;
	}
};

#endif