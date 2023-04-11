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
};

#endif