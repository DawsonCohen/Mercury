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

	friend void swap(Element& e1, Element& e2) {
        using std::swap;
        swap(e1.masses, e2.masses);
        swap(e1.springs, e2.springs);
        swap(e1.faces, e2.faces);
        swap(e1.cells,e2.cells);
        swap(e1.boundaryCount,e2.boundaryCount);
        swap(e1.sim_time,e2.sim_time);
        swap(e1.total_sim_time,e2.total_sim_time);
    }
};

#endif