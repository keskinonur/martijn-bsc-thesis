#include "rl_forcefield.h"


rl_forcefield::rl_forcefield(double width, double height, double coarse)
{
	// force field setup
	int hor, ver;

	hor = (int) floor((width-0.5*coarse) / coarse);
	ver = (int) floor((height-0.5*coarse) / coarse);

	particle_count = hor * ver;
	particles = new rl_particle[particle_count];

	// set default values for particles and pointers to neighbours
	for (int i=0; i<hor; i++) {
		for (int j=0; j<ver; j++) {
			int curr = j*hor + i;
			particles[curr].loc[0] = 0.5*coarse + i * coarse;
			particles[curr].loc[1] = 0.5*coarse + j * coarse;
			particles[curr].vect[0] = 0;
			particles[curr].vect[1] = 0;
			particles[curr].val = 0;
			particles[curr].top    = (j>0)     ? (&particles[curr-hor]) : NULL;
			particles[curr].right  = (i<hor-1) ? (&particles[curr+1])   : NULL;
			particles[curr].bottom = (j<ver-1) ? (&particles[curr+hor]) : NULL;
			particles[curr].left   = (i>0)     ? (&particles[curr-1])   : NULL;
		}
	}
	
}


rl_forcefield::~rl_forcefield(void)
{
}


rl_particle *rl_forcefield::get_nearest_particle(double loc[2], rl_particle *start) {
	// search for nearest particle for location in 'vect'
	// start with particle 'start'
	double d = dist(loc, start->loc);
	rl_particle *curr = start;
	bool changed = true;
	while (changed) {
		changed = false;
		if (curr->top != NULL && dist(loc, curr->top->loc) < d) {
			d = dist(loc, curr->top->loc);
			curr = curr->top;
			changed = true;
		} else if (curr->right != NULL && dist(loc, curr->right->loc) < d) {
			d = dist(loc, curr->right->loc);
			curr = start->right;
			changed = true;
		} else if (curr->bottom != NULL && dist(loc, curr->bottom->loc) < d) {
			d = dist(loc, curr->bottom->loc);
			curr = curr->bottom;
			changed = true;
		} else if (curr->left != NULL && dist(loc, curr->left->loc) < d) {
			d = dist(loc, curr->left->loc);
			curr = curr->left;
			changed = true;
		}
	}
	return curr;
}