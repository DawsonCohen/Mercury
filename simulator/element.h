#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "mass.h"
#include "spring.h"

struct Element {
	std::vector<Mass> masses;
	std::vector<Spring> springs;

	float sim_time = 0;
	float total_sim_time = 0;
};

#endif