#ifndef __SPRING_H__
#define __SPRING_H__
#include <vector>
#include "mass.h"
#include "stdio.h"
#include "math.h"

struct Spring {
	uint m0;
	uint m1;
	float rest_length;
	float mean_length;
	Material material;

	friend std::ostream& operator<<(std::ostream& out, const Spring& s) {
		return out << s.m0 << "," << s.m1 << "," << s.rest_length << "," << s.mean_length << "," << (unsigned int) s.material.encoding;
	}
};

#endif