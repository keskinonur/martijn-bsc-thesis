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

// there is just one stage transition
void rl_stage::add_stage_transition(double v1[2], double v2[2], rl_stage *stage) {
	cp(v1, tr->loc1);
	cp(v2, tr->loc2);
	tr->stage = stage;
}

void rl_stage::save_to_file(char *filename, double loc[2], rl_particle *part_last) {
	/* Format:
	 * particle per line:
	 * x y vect_x vect_y val
	 */
	ofstream fp;
	fp.open(filename);
	fp << loc[0] << " " << loc[1] << endl;
	fp << part_last->loc[0] << " " << part_last->loc[1] << " " << part_last->vect[0] << " " << part_last->vect[1] << endl;
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

bool rl_stage::check_for_stage_transition(double v1[2], double v2[2])
{
	double *a = tr->loc1;
	double *b = v2;
	double *c = tr->loc2;
	double *d = v1;
	if (sign(a, b, c) == sign(b, c, d) 
	 && sign(b, c, d) == sign(c, d, a)
	 && sign(c, d, a) == sign(d, a, b)) {
		printf("a = %f %f\n", a[0], a[1]);
		printf("b = %f %f\n", b[0], b[1]);
		printf("c = %f %f\n", c[0], c[1]);
		printf("d = %f %f\n", d[0], d[1]);

		cout << sign(a, b, c) << sign(b, c, d) << sign(c, d, a) << sign(d, a, b);
		return true;
	} else {
		return false;
	}
}
