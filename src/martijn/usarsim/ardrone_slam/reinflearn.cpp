#include "global.h"
#include "reinflearn.h"
#include "bot_ardrone.h"
#include <io.h>
#include <math.h>

#include <cv.hpp>



reinflearn::reinflearn(bot_ardrone *bot)
{
	printf("REINF STARTED\n");

	rl_bot = bot;

	// define level constants
	layout = new rl_layout;
	layout->width  = 2 * 9.75;
	layout->height = 2 * 7.25;
	layout->offset[0] = -9.75;
	layout->offset[1] = -7.25;
	layout->pylon1[0] = 4.5;
	layout->pylon1[1] = 0.1;
	layout->pylon1[0] = -5.1;
	layout->pylon1[1] = 0.1;
	layout->pylonr = 0.45;
	layout->coarse = 1.0;
	

	

	Sleep(3000);


	// Create stages
	rl_stage *stage_l2r = new rl_stage(this);
	rl_stage *stage_r2l = new rl_stage(this);
	double v1[2], v2[2]; // v1: pylon, v2: wall
	real2sim(layout->pylon1, v1);
    v2[0] = layout->width;
	v2[1] = v1[1];
	stage_l2r->add_stage_transition(v1, v2, stage_r2l);
	real2sim(layout->pylon2, v1);
    v2[0] = 0;
	v2[1] = v1[1];
	stage_r2l->add_stage_transition(v1, v2, stage_l2r);
	stage_curr = stage_l2r;



	// some sandbox testing code
	
	stage_curr->save_to_file("../../particles.txt");

	double t = 0.0;
	double vect_fly[2];
	double vect_loc[2];
	bot_ardrone_measurement *m = new bot_ardrone_measurement;
	rl_particle *part_last = &(stage_curr->ff->particles[0]);

	while (1) {

		t = (t + PI/18);
		vect_fly[0] = cos(t);
		vect_fly[1] = sin(t);
		fly_vector(vect_fly);
		
		bot->recorder->get_last(m);
		vect_loc[0] = (double) m->gt_loc[0];
		vect_loc[1] = (double) m->gt_loc[1];
		real2sim(vect_loc, vect_loc);

		part_last = stage_curr->ff->get_nearest_particle(vect_loc, part_last);

		printf("location: %f, %f; nearest particle: %f, %f\n",
			   vect_loc[0],
			   vect_loc[1],
			   part_last->loc[0],
			   part_last->loc[1]);

		Sleep(150);
	}

	// TODO TODO TODO

}

reinflearn::~reinflearn()
{
}

void reinflearn::control_set(int type, double velocity) {

	if (rl_bot->control_get(BOT_ARDRONE_Velocity, type) != velocity)
	{
		rl_bot->control_set(BOT_ARDRONE_Velocity, type, (float)velocity);
		rl_bot->control_update();
	}

}

void reinflearn::fly_vector(double vect[2]) {
	// vect[0]: linear (to front/back); vect[1]: lateral (to right/left)

	rl_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, (float)vect[0]);
	rl_bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, (float)vect[1]);
	rl_bot->control_update();

}

// from unreal-level to this simulation
void reinflearn::real2sim(double vect_in[2], double vect_out[2]) {
	vect_out[0] = vect_in[0] - layout->offset[0];
	vect_out[1] = vect_in[1] - layout->offset[1];
}

// from this simulation to unreal-level
void reinflearn::sim2real(double vect_in[2], double vect_out[2]) {
	vect_out[0] = vect_in[0] + layout->offset[0];
	vect_out[1] = vect_in[1] + layout->offset[1];
}