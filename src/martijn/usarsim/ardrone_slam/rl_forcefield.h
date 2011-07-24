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
	rl_particle *get_matching_particle(rl_particle *start);
	double fdv(double distance, double maximum); // force distance value (don't ask)
	void add_rect_forces(double loc1[2], double loc2[2], double pwr, double range);
	void add_circle_forces(double loc[2], double radius, double pwr, double range);
	void add_spiral_forces(double loc[2], double radius, double pwr, double range, double angle);
	void add_attract_forces(double loc[2], double pwr);
};

