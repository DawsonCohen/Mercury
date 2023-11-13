#ifndef __STRUCTS_H__
#define __STRUCTS_H__

enum RobotType {
	ROBOT_NN,
	ROBOT_VOXEL
};

enum EnvironmentType {
    ENVIRONMENT_LAND,
    ENVIRONMENT_WATER
};

enum CrossoverDistribution {
	CROSS_DIST_NONE = 0,
	CROSS_DIST_BINOMIAL = 1
};

enum CrossoverType {
	CROSS_INDIVIDUAL = 0,
	CROSS_CONTIGUOUS = 1
};

#endif