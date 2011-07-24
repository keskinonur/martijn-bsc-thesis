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
	layout->pylon1[0] = -5.1;
	layout->pylon1[1] = 0.1;
	layout->pylon2[0] = 4.5;
	layout->pylon2[1] = 0.1;
	layout->pylonr = 0.45;
	layout->coarse = 0.6;
	layout->history = 10;
	

	
	//Sleep(3000);


	// Create stages
	rl_stage *stage_l2r = new rl_stage(this);
	rl_stage *stage_r2l = new rl_stage(this);
	double v1[2], v2[2]; // v1: pylon, v2: wall
	real2sim(layout->pylon1, v1);
    v2[0] = 0;
	v2[1] = v1[1];
	stage_r2l->add_stage_transition(v1, v2, stage_l2r);
	real2sim(layout->pylon2, v1);
    v2[0] = layout->width;
	v2[1] = v1[1];
	stage_l2r->add_stage_transition(v1, v2, stage_r2l);
	stage_curr = stage_l2r;

	// Set initial force field
	// - wall
	v1[0] = 0;
	v1[1] = 0;
	v2[0] = layout->width;
	v2[1] = layout->height;
	stage_l2r->ff->add_rect_forces(v1, v2, 1, 5);
	stage_r2l->ff->add_rect_forces(v1, v2, 1, 5);
	// - pylon1
	real2sim(layout->pylon1, v1);
	stage_l2r->ff->add_circle_forces(v1, layout->pylonr, 1, 10);
	stage_r2l->ff->add_circle_forces(v1, layout->pylonr, 1, 10);
	stage_l2r->ff->add_spiral_forces(v1, layout->pylonr, 1, 10, PI/2);
	stage_r2l->ff->add_spiral_forces(v1, layout->pylonr, 1, 10, PI/2);
	// - pylon2
	real2sim(layout->pylon2, v1);
	stage_l2r->ff->add_circle_forces(v1, layout->pylonr, 1, 10);
	stage_r2l->ff->add_circle_forces(v1, layout->pylonr, 1, 10);
	stage_l2r->ff->add_spiral_forces(v1, layout->pylonr, 1, 10, -PI/2);
	stage_r2l->ff->add_spiral_forces(v1, layout->pylonr, 1, 10, -PI/2);
	// attract
	real2sim(layout->pylon1, v1);
	stage_r2l->ff->add_attract_forces(v1, 0.5);
	real2sim(layout->pylon2, v1);
	stage_l2r->ff->add_attract_forces(v1, 0.5);

	printf("%f %f and %f %f\n", stage_curr->tr->loc1[0], stage_curr->tr->loc1[1], stage_curr->tr->loc2[0], stage_curr->tr->loc2[1]);


	// some sandbox testing code
	
	double vect_fly[2];
	double loc_curr[2];
	double loc_last[2];
	long time_now;
	long time_last = 0;
	bot_ardrone_measurement *m = new bot_ardrone_measurement;
	rl_particle *part_last = &(stage_curr->ff->particles[0]);

	Sleep(500);


	while (1) {

		bot->recorder->get_last(m);
		loc_curr[0] = (double) m->gt_loc[0];
		loc_curr[1] = (double) m->gt_loc[1];
		real2sim(loc_curr, loc_curr);
		time_now = timestamp_millis();

		if (time_last != 0) {
			if (stage_curr->check_for_stage_transition(loc_curr, loc_last)) {
				// stage transition
				printf("Stage transition! %p -> %p\n", stage_curr, stage_curr->tr->stage);
				printf("%f %f -> %f %f\n", loc_last[0], loc_last[1], loc_curr[0], loc_curr[1]);
				stage_curr = stage_curr->tr->stage;
				part_last = stage_curr->ff->get_matching_particle(part_last);
			}
		}

		part_last = stage_curr->ff->get_nearest_particle(loc_curr, part_last);

		// save for visualisation
		stage_curr->save_to_file("../../particles.txt", loc_curr, part_last);


		// fly command
		/*if ((double)rand()/RAND_MAX < 0.001) {
			vect_fly[0] = 6*((double)rand()/RAND_MAX - 0.5);
			vect_fly[1] = 6*((double)rand()/RAND_MAX - 0.5);
			printf("Explore! %f %f\n", vect_fly[0], vect_fly[1]);
		} else {
		*/
			cp(part_last->vect, vect_fly);
			printf("Force:   %f %f\n", vect_fly[0], vect_fly[1]);
		//}
		
		unit(vect_fly, vect_fly); // constant speed
		vect_fly[0] *= 4;
		vect_fly[1] *= 4;
		fly_vector(vect_fly);

		loc_last[0] = loc_curr[0];
		loc_last[1] = loc_curr[1];
		time_last = timestamp_millis();



		/*
		printf("loc: %f, %f; part: %f, %f; fly: %f %f \n",
			   loc_curr[0],
			   loc_curr[1],
			   part_last->loc[0],
			   part_last->loc[1],
			   vect_fly[0],
			   vect_fly[1]);
		/* */

		Sleep(15); // improves speed estimation
	}

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

	// rotate absolute vector 'vect' back gt_or
	bot_ardrone_measurement *m = new bot_ardrone_measurement;
	rl_bot->recorder->get_last(m);
	rot(vect, -1*m->gt_or[0], vect);
	// give command to robot
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



// Helper functions
long reinflearn::timestamp_millis() {
	return (long)GetTickCount64();

	//SYSTEMTIME *tm;
	//GetSystemTime(tm);
	//return (24*60*1000*(int)tm->wDay + 60*1000*(int)tm->wHour + 1000 * (int)tm->wSecond + (int)tm->wMilliseconds);
}