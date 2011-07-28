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
	layout->epsilon = 0.1;

	bool restore = false;
	
	stats = new rl_stats;
	stats->filename = "../../stats.txt";
	stats->last_transition = 0;
	stats->number_of_transitions = 0;

	//history = new rl_history[layout->history];
	history_clear();

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

	if (restore) {
		
		// restore force fields from files
		stage_r2l->load_field_from_file("../../stage_r2l.txt");
		stage_l2r->load_field_from_file("../../stage_l2r.txt");

	} else {
	
		// Set initial force fields
		
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

	}

	// some sandbox testing code
	
	double vect_fly[2];
	double loc_curr[2];
	double loc_last[2];
	long time_now;
	long time_last = 0;
	bot_ardrone_measurement *m = new bot_ardrone_measurement;
	rl_particle *part_last = &(stage_curr->ff->particles[0]);

	Sleep(500);

	stats->last_transition = (long) time(NULL);

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
				part_last = stage_curr->ff->get_matching_particle(part_last->loc[0], part_last->loc[1]);
				// update stats
				save_stats_to_file(stats);
				stats->last_transition = (long) time(NULL);
				stats->number_of_transitions++;
				history_clear();
			}
		}

		part_last = stage_curr->ff->get_nearest_particle(loc_curr, part_last);


		// reinforcement learning
		// ...


		// save for visualisation
		stage_curr->save_to_file("../../particles.txt", loc_curr, part_last);
		// save for restore
		stage_r2l->save_field_to_file("../../stage_r2l.txt", "../../rm stage_r2l_tmp.txt");
		stage_l2r->save_field_to_file("../../stage_l2r.txt", "../../stage_l2r_tmp.txt");

		// fly command
		if ( (history[0] == NULL)
		  || (part_last->loc[0] != history[0]->particle->loc[0])
		  || (part_last->loc[1] != history[0]->particle->loc[1]) ) {
			// new particle, determine if we will explore or use policy
			if ((double)rand()/RAND_MAX < layout->epsilon) {
				add_to_history(part_last, true);
				cp(history[0]->explore_vect, vect_fly);
				printf("Explore! %f %f\n", vect_fly[0], vect_fly[1]);
			} else {
				add_to_history(part_last, false);
				cp(history[0]->particle->vect, vect_fly);
				printf("Force:   %f %f\n", vect_fly[0], vect_fly[1]);
			}			
		}
		// else: same particle, thus same command as last time
		
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



// history
void reinflearn::history_clear() {
	for (int i=0; i<layout->history; i++) {
		history[i] = NULL;
	}
}

// adds pointers to original particles to the history
// will make random explore vector when necessary
void reinflearn::add_to_history(struct rl_particle *particle, bool explore) {
	// move older particles
	for (int i=layout->history - 2; i>0; i--) {
		if (history[i] == NULL)
			continue;
		history[i] = history[i-1];
	}
	// add particle
	history[0] = new rl_history;
	history[0]->particle = particle;
	history[0]->explore = explore;
	if (explore) {
		double weight = 2;
		double expl[2];
		expl[0] = ((double)rand()/RAND_MAX - 0.5);
		expl[1] = ((double)rand()/RAND_MAX - 0.5);
		unit(expl, expl);
		expl[0] *= weight;
		expl[1] *= weight;

		history[0]->explore_vect[0] = expl[0];
		history[0]->explore_vect[1] = expl[1];
	} else {
		history[0]->explore_vect[0] = 0;
		history[0]->explore_vect[1] = 0;
	}
	history[0]->speed[0] = 0;
	history[0]->speed[1] = 0;
}


// Helper functions
long reinflearn::timestamp_millis() {
	return (long)GetTickCount64();

	//SYSTEMTIME *tm;
	//GetSystemTime(tm);
	//return (24*60*1000*(int)tm->wDay + 60*1000*(int)tm->wHour + 1000 * (int)tm->wSecond + (int)tm->wMilliseconds);
}

void reinflearn::save_stats_to_file(rl_stats *stats) {
	long diff = (long)time(NULL) - stats->last_transition;
	
	fstream fp;
	fp.open(stats->filename, fstream::in | fstream::out | fstream::app);
	fp << stats->number_of_transitions << " " << diff << endl;
	fp.close();
}

