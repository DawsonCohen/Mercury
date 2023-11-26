#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "mass.h"
#include "spring.h"
#include "face.h"
#include "cell.h"

struct Element {
	std::vector<Mass> masses;
	std::vector<Spring> springs;
	std::vector<Face> faces;
	std::vector<Cell> cells;

	unsigned int boundaryCount = 0;

	float sim_time = 0;
	float total_sim_time = 0;
};

#endif