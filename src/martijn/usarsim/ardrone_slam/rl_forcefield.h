#pragma once

#include "reinflearn.h"

class rl_forcefield
{
public:
	struct rl_particle *particles;
	int particle_count;
	rl_forcefield(double width, double height, double coarse);
	~rl_forcefield(void);

	rl_particle *get_nearest_particle(double vect[2], rl_particle *start);
};

