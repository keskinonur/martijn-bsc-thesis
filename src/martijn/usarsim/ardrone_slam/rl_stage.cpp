#include "rl_stage.h"


rl_stage::rl_stage(reinflearn *rl)
{
	this->rl = rl;
	this->tr = new rl_transition;
	this->ff = new rl_forcefield(rl->layout->width, rl->layout->height, rl->layout->coarse);
}


rl_stage::~rl_stage(void)
{
}


void rl_stage::add_stage_transition(double v1[2], double v2[2], rl_stage *stage) {
	cp(v1, tr->loc1);
	cp(v2, tr->loc2);
	tr->stage = stage;
}

void rl_stage::save_to_file(char *filename) {
	/* Format:
	 * particle per line:
	 * x y vect_x vect_y val
	 */
	ofstream fp;
	fp.open(filename);
	for (int i=0; i<ff->particle_count; i++) {
		fp << ff->particles[i].loc[0] << " ";
		fp << ff->particles[i].loc[1] << " ";
		fp << ff->particles[i].vect[0] << " ";
		fp << ff->particles[i].vect[1] << " ";
		fp << ff->particles[i].val;
		fp << endl;
	}
	fp.close();
}